
#include "OpenStaServerHandler.hpp"

#include "StaConfig.hh"
#include "Sta.hh"
#include "VerilogReader.hh"
#include "Network.hh"
#include "ConcreteLibrary.hh"
#include "ConcreteNetwork.hh"
#include "PortDirection.hh"
#include "Graph.hh"
#include "Search.hh"
#include "SearchClass.hh"
#include "PathVertex.hh"
#include "TimingArc.hh"
#include "PathRef.hh"
#include "DcalcAnalysisPt.hh"
#include "TagGroup.hh"

#include "Sdc.hh"
#include "Parasitics.hh"

#include <deque>
#include <iostream>
#include <cstring>
#include <unordered_set>


namespace stamask {



OpenStaServerHandler::OpenStaServerHandler() {
	mTclInterp = Tcl_CreateInterp();
    if (Tcl_Init(mTclInterp) != TCL_OK)
        throw std::runtime_error("Failed to init TCL interpreter");

    sta::initSta();
    mStaPtr = new sta::Sta;
    sta::Sta::setSta(mStaPtr);
    mStaPtr->makeComponents();
    mStaPtr->setTclInterp(mTclInterp);
    int thread_count = 1;
    mStaPtr->setThreadCount(thread_count);

    mExecResultMessage = "";
}


OpenStaServerHandler::~OpenStaServerHandler() {
	sta::deleteAllMemory();
}

/**
 * Returns message that was set up after command execution.
 * @return execution message
 */
std::string OpenStaServerHandler::getExecMessage() const {
	return mExecResultMessage;
}


bool OpenStaServerHandler::execute(
		const CommandSetHierarhySeparator& inCommand) {
	setExecMessage("");
	if(inCommand.mStr.length() == 0 ||
			inCommand.mStr.length() > 1) {
		setExecMessage("need string with single character");
		return false;
	}

	if(!checkStaReady())
		return false;

	if(sta::Sta::sta()->network())
		sta::Sta::sta()->network()->setPathDivider(inCommand.mStr.at(0));

	return true;
}

/**
 * Command to clean up data and exit.
 */
bool OpenStaServerHandler::execute(
		const CommandExit& inCommand) {
	setExecMessage("");
	return true;
}

/**
 * Command to return status that OpenSTA is ready.
 */
bool OpenStaServerHandler::execute(
		const CommandPing& inCommand) {
	setExecMessage("");
	return sta::Sta::sta() != nullptr;
}

/**
 * Command reads liberty and adds new library in OpenSTA's network and cmd_corner.
 * KNOWN BUGS: OpenSTA piles up all libraries even if their names and files are the same.
 * This leads allocated memory to bloat up.
 */
bool OpenStaServerHandler::execute(
		const CommandReadLibertyFile& inCommand) {
	setExecMessage("");
	if(!checkStaReady())
		return false;

	sta::LibertyLibrary* libPtr = sta::Sta::sta()->readLiberty(
			inCommand.mStr.c_str(),
			sta::Sta::sta()->cmdCorner(),
			sta::MinMaxAll::all(), true);

	if(!libPtr)
		setExecMessage("failed to read liberty " + inCommand.mStr);

	return libPtr != nullptr;
}

/**
 * Does nothing and returns false.
 */
bool OpenStaServerHandler::execute(
		const CommandReadLibertyStream& inCommand) {
	setExecMessage("cannot read liberty from string");
	return false;
}

/**
 * Clears network and STA.
 * Returns false if STA wasn't set up.
 */
bool OpenStaServerHandler::execute(
		const CommandClearLibs& inCommand) {
	setExecMessage("");
	if(!checkStaReady())
		return false;

	sta::Sta *sta = sta::Sta::sta();
	sta->network()->clear();
	sta->clear();

	return true;
}


/**
 * Reads verilog netlist from file.
 * Returns false if sta wasn't set up or verilog wasn't read.
 */
bool OpenStaServerHandler::execute(
		const CommandReadVerilogFile& inCommand) {
	setExecMessage("");

	if(!checkStaReady())
		return false;

	sta::NetworkReader *networkReaderPtr = sta::Sta::sta()->networkReader();
	if(!networkReaderPtr) {
		setExecMessage("failed to get network reader");
		return false;
	}

	if(networkReaderPtr->report())
		networkReaderPtr->report()->redirectStringBegin();

	sta::Sta::sta()->readNetlistBefore();
	bool isOk = sta::readVerilogFile(inCommand.mStr.c_str(), networkReaderPtr);

	if(networkReaderPtr->report())
		setExecMessage(networkReaderPtr->report()->redirectStringEnd());
	else
		setExecMessage("");

	return isOk;
}

/**
 * Does nothing and return false.
 */
bool OpenStaServerHandler::execute(
		const CommandReadVerilogStream& inCommand) {
	setExecMessage("cannot read verilog from string");
	return false;
}

/**
 * Links top-level block of previously read verilog description.
 * Returns false if sta wasn't set up or failed to link design.
 */
bool OpenStaServerHandler::execute(
		const CommandLinkTop& inCommand) {
	setExecMessage("");

	setExecMessage("");
	if(!checkStaReady())
		return false;

	sta::Sta* sta = sta::Sta::sta();
	if(sta::Sta::sta()->report())
		sta::Sta::sta()->report()->redirectStringBegin();

	bool isOk = sta->linkDesign(inCommand.mStr.c_str());

	if(sta::Sta::sta()->report())
		setExecMessage(sta::Sta::sta()->report()->redirectStringEnd());

	return isOk;
}


/**
 * Clears read netlist and top-instance if they exist.
 * Returns false if sta wasn't set up.
 */
bool OpenStaServerHandler::execute(
		const CommandClearNetlistBlocks& inCommand) {
	setExecMessage("");
	if(!checkStaReady())
		return false;

	sta::Sta::sta()->readNetlistBefore();
	sta::Sta::sta()->networkChanged();

	return true;
}

/**
 * Clears previous netlist, creates new one and links top-instance.
 * Returns false if Sta isn't ready, no top-block or
 * fails to create library or netlist.
 */
bool OpenStaServerHandler::execute(
						const CommandCreateNetlist& inCommand) {
	setExecMessage("");
	if(!checkStaReady())
		return false;

	//getting index of top-module
	uint32_t topBlockIdx = findTopBlockIdx(inCommand.mBlockDataVec);
	//nothing to do without the top
	if(topBlockIdx >= inCommand.mBlockDataVec.size()) {
		setExecMessage("failed to find top-level block");
		return false;
	}

	//getting network
	sta::Sta *sta = sta::Sta::sta();
	sta::NetworkReader* networkPtr = sta->networkReader();

	//have to clear the previous design before adding new one
	sta::Sta::sta()->readNetlistBefore();

	//locating target netlist library
	sta::Library* netlistLibPtr = getCreateNetlistLib(networkPtr);
	if(!netlistLibPtr) {
		setExecMessage("failed to locate library for netlist");
		return false;
	}

	//first creating dummy declaration for top-cell
	sta::Cell *cell = findCreateBlockDeclaration(
			networkPtr, netlistLibPtr,
			inCommand.mBlockDataVec[topBlockIdx]);
	if(!cell) {
		setExecMessage("couldn't create top block declaration");
		return false;
	}

	//creating top-instance
	sta::Instance* topInstPtr =
			networkPtr->makeInstance(cell, "", nullptr);

	//then recursively filling the block's body,
	//keeping context of the instance hierarchy
	//all exec messages are prepared inside the method call
	if(!createBlockBody(
			networkPtr, netlistLibPtr,
			nullptr, BlockData(),
			topInstPtr, inCommand.mBlockDataVec[topBlockIdx],
			cell, inCommand.mBlockDataVec, {})) {
		addExecMessage("failed to create body of top-instance");
		return false;
	}

	//for now setting top-instance through network casting
	sta::ConcreteNetwork* cNetworkPtr =
			reinterpret_cast<sta::ConcreteNetwork*>(networkPtr);
	cNetworkPtr->setTopInstance(topInstPtr);

	return true;
}

/**
 * Collects timing graph data and writes out vertext and edge information.
 * Returns false if Sta or graph aren't ready.
 * First collects vertex data, then edge data.
 * @param inCommand command to execute
 * @param outVertexIdToDataVec container for output vertex data
 * @param outEdgeIdToDataVec container for output edge data
 * @return success flag
 */
bool OpenStaServerHandler::execute(
						const CommandGetGraphData& inCommand,
						std::vector<VertexIdData>& outVertexIdToDataVec,
						std::vector<EdgeIdData>& outEdgeIdToDataVec) {
	setExecMessage("");
	if(!checkDesignLoaded())
		return false;

	sta::Network* networkPtr = sta::Sta::sta()->cmdNetwork();
	sta::Graph* graphPtr = sta::Sta::sta()->ensureGraph();
	if(!graphPtr) {
		setExecMessage("got null timing graph");
		return false;
	}

	//allocating data
	outVertexIdToDataVec.clear();
	outEdgeIdToDataVec.clear();

	//collecting vertexes and remapping their IDs
	std::vector<uint32_t> vertexIdToIdxVec;
	collectVertexesData(
			networkPtr, graphPtr,
			vertexIdToIdxVec,
			outVertexIdToDataVec);

	//filling all edges
	//doing so by iterating over all outgoing edges from each vertex
	//graph doesn't have edge iterator, and edge container is multilayered
	//so it's a little unsafe to use indexes
	collectEdgesData(graphPtr, vertexIdToIdxVec, outEdgeIdToDataVec);

	return true;
}

/**
 * Collects vertex data from timing graph and remaps their IDs.
 * Does nothing and returns false if either input pointer is null.
 * Ignores vertexes without associated pin.
 * @param inNetworkPtr target network
 * @param inGraphPtr timing graph
 * @param outVertexIdToIdxVec vertex remapping
 * @param outVertexIdToDataVec output container for vertex data
 * @return success flag
 */
bool OpenStaServerHandler::collectVertexesData(
		sta::Network* inNetworkPtr,
		sta::Graph* inGraphPtr,
		std::vector<uint32_t>& outVertexIdToIdxVec,
		std::vector<VertexIdData>& outVertexIdToDataVec) {
	if(!inNetworkPtr || !inGraphPtr)
		return false;

	outVertexIdToIdxVec.clear();
	outVertexIdToIdxVec.resize(
			inGraphPtr->vertexCount(),
			std::numeric_limits<uint32_t>::max());

	sta::Vertex* vertexPtr = nullptr;
	sta::Pin* pinPtr = nullptr;

	//setting vertex data
	sta::VertexIterator vertexIt(inGraphPtr);
	while(vertexIt.hasNext()) {
		vertexPtr = vertexIt.next();
		if(!vertexPtr)
			continue;

		//ignoring pinless vertexes
		pinPtr = vertexPtr->pin();
		if(!pinPtr)
			continue;

		//mapping vertex ID to index
		if(vertexPtr->objectIdx() >= outVertexIdToIdxVec.size())
			outVertexIdToIdxVec.resize(
					vertexPtr->objectIdx()+1,
					std::numeric_limits<uint32_t>::max());

		outVertexIdToIdxVec[vertexPtr->objectIdx()] = outVertexIdToDataVec.size();

		outVertexIdToDataVec.push_back(VertexIdData());
		outVertexIdToDataVec.back().mVertexId = vertexPtr->objectIdx();
		outVertexIdToDataVec.back().mIsDriver =
				vertexPtr->isDriver(inNetworkPtr) ||
				vertexPtr->isBidirectDriver();
		fillPinPaths(
				inNetworkPtr, pinPtr,
				outVertexIdToDataVec.back().mPinName,
				outVertexIdToDataVec.back().mContextInstNamesVec);
	}

	return true;
}

/**
 * Collects edge data from timing graph, edges have vertex index not ID from graph.
 * Does nothing and returns false if graph input pointer is null.
 * Ignores edge without source/target vertexes or with vertexes with invalid IDs.
 * Preserves original edge IDs, because they are needed to mess with their delays on PnR.
 * @param inGraphPtr timing graph
 * @param inVertexIdToIdxVec vertex remapping
 * @param outEdgeIdToDataVec output container for edge data
 * @return success flag
 */
bool OpenStaServerHandler::collectEdgesData(
		sta::Graph* inGraphPtr,
		const std::vector<uint32_t>& inVertexIdToIdxVec,
		std::vector<EdgeIdData>& outEdgeIdToDataVec) {
	if(!inGraphPtr)
		return false;

	sta::Vertex* anchorVertexPtr = nullptr;
	sta::Edge* edgePtr = nullptr;
	sta::Vertex* fromPtr = nullptr;
	sta::Vertex* toPtr = nullptr;
	uint32_t fromId = 0;
	uint32_t toId = 0;

	//setting edge data
	//edges enumerate from the index=1, because sta::object_id_null (0) isn't occupied
	//so shifting upper limit to hit the edgeCount()

	//NOTE: internal table is two-dimensional, edges contain index in the second dimension
	//so several edges may result with the same objectId, this may be frustrating...

	//NOTE2: it is safer for iterate through vertexes, I'm not sure it's completely safe to use [1, count]
	//because author didn't use it much himself

	//NOTE3: duplicase edges between the same pair of nodes
	//are the liberty arcs from timing() with different operation conditions

	sta::VertexIterator vertexIt2(inGraphPtr);
	while(vertexIt2.hasNext()) {
		anchorVertexPtr = vertexIt2.next();
		if(!anchorVertexPtr)
			continue;

		//std::cout << "edges from vertex " << anchorVertexPtr->objectIdx() << ":" << std::endl;

		sta::VertexOutEdgeIterator edge_iter(anchorVertexPtr, inGraphPtr);
		while (edge_iter.hasNext()) {
			edgePtr = edge_iter.next();
			if(!edgePtr)
				continue;

			fromPtr = edgePtr->from(inGraphPtr);
			toPtr = edgePtr->to(inGraphPtr);
			if(!fromPtr || !toPtr)
				continue;

			if(fromPtr->objectIdx() >= inVertexIdToIdxVec.size() ||
					toPtr->objectIdx() >= inVertexIdToIdxVec.size())
				continue;

			fromId = inVertexIdToIdxVec[fromPtr->objectIdx()];
			toId = inVertexIdToIdxVec[toPtr->objectIdx()];

			outEdgeIdToDataVec.push_back(EdgeIdData());
			//WARNING: do not use edgePtr->objectIdx(),
			//because it's return value may not match the real ID
			outEdgeIdToDataVec.back().mEdgeId = inGraphPtr->id(edgePtr);
			outEdgeIdToDataVec.back().mFromVertexId = fromId;
			outEdgeIdToDataVec.back().mToVertexId = toId;
		}
	}

	return true;
}


/**
 * Collects AAT and RAT of timing nodex.
 * Does nothing and returns false if sta or graph aren't ready.
 * First adds AAT and RAT of nodes.
 * Then traverses backward from all endpoints to mark endpoint RAT for nodes in timing paths.
 * @param inCommand command to execute
 * @param outNodeTimingsVec container for output timing data
 * @return success code
 */
bool OpenStaServerHandler::execute(
						const CommandGetGraphSlacksData& inCommand,
						std::vector<NodeTimingData>& outNodeTimingsVec) {
	setExecMessage("");
	if(!checkDesignLoaded())
		return false;

	sta::Graph* graphPtr = sta::Sta::sta()->ensureGraph();
	if(!graphPtr) {
		setExecMessage("timing graph is null");
		return false;
	}

	sta::Sta::sta()->updateTiming(false);//searchPreamble();

	//refresh or calculate anew RATs/AATs in the graph
	sta::Search* searchPtr = sta::Sta::sta()->search();
	searchPtr->findAllArrivals();
	searchPtr->findRequireds();


	outNodeTimingsVec.clear();
	outNodeTimingsVec.resize(graphPtr->vertexCount());
	sta::Vertex* vertexPtr = nullptr;

	//process all the timing nodes and fill the vertex timings
	for(sta::VertexId vId = 0; vId < graphPtr->vertexCount(); vId++) {
		outNodeTimingsVec[vId].mHasEndMaxPathRat = false;
		outNodeTimingsVec[vId].mHasEndMinPathRat = false;
		outNodeTimingsVec[vId].mHasTiming = false;

		vertexPtr = graphPtr->vertex(vId);
		if(!vertexPtr)
			continue;

		fillGraphNodeAatRat(sta::Sta::sta(), vertexPtr, outNodeTimingsVec[vId]);
	}

	//get all timing path endpoints, for each endpoint
	//trace back all incoming nodes with rat+aat to set path endpoint rat
	sta::VertexSet* endpointSetPtr = sta::Sta::sta()->search()->endpoints();
	if(!endpointSetPtr) {
		setExecMessage("failed to get timing endpoints");
		return false;
	}

	size_t idx = 0;
	for(auto vertexIt = endpointSetPtr->begin(),
			vertexEndIt = endpointSetPtr->end();
			vertexIt != vertexEndIt; vertexIt++) {
		if(!*vertexIt)
			continue;

		//if vertex is an end-check of timing path,
		//then have to collect all previous vertexes that are affected by this RAT
		if((!(*vertexIt)->isConstrained() &&
				!(*vertexIt)->hasChecks()) ||
				!(*vertexIt)->hasRequireds())
			continue;

		//separately traversing back paths for min and max constraint
		fillSetBackpathNodesRat(
				sta::Sta::sta(), graphPtr, *vertexIt,
				outNodeTimingsVec, idx, true);
		fillSetBackpathNodesRat(
				sta::Sta::sta(), graphPtr, *vertexIt,
				outNodeTimingsVec, idx, false);
		idx++;
	}

	return true;
}


/**
 * Fills required and arrival times of the node in graph.
 * Does nothing and returns false if either input pointer is null.
 * @param inStaPtr target Sta
 * @param inVertexPtr target vertex
 * @param outNodeData data to fill
 * @return success flag
 */
bool OpenStaServerHandler::fillGraphNodeAatRat(
						sta::Sta* inStaPtr,
						sta::Vertex* inVertexPtr,
						NodeTimingData& outNodeData) {
	if(!inStaPtr || !inVertexPtr)
		return false;

	outNodeData.mNodeId = inVertexPtr->objectIdx();

	sta::Arrival aatValue = 0;
	sta::Required ratValue = 0;
	sta::Slack worstMinSlack = 1e30;
	sta::Slack worstMaxSlack = 1e30;

	//processing all min/max rise/fall timings of the node
	sta::VertexPathIterator path_iter(inVertexPtr, inStaPtr);
    while (path_iter.hasNext()) {
    	sta::PathVertex *path = path_iter.next();
    	if(!path)
    		continue;

     	aatValue = path->arrival(inStaPtr);
     	ratValue = path->required(inStaPtr);
     	sta::Slack slack = path->slack(inStaPtr);

     	//for min/max constraints setting aat/rat values if it's slack < current worst
     	if(path->minMax(inStaPtr) == sta::MinMax::max() &&
     			slack < worstMaxSlack) {
     		outNodeData.mMaxWorstSlackAat = aatValue;
     		outNodeData.mMaxWorstSlackRat = ratValue;
     		outNodeData.mHasTiming = true;
     		continue;
     	}

     	if(path->minMax(inStaPtr) == sta::MinMax::min() &&
     			slack < worstMinSlack) {
     		outNodeData.mMinWorstSlackAat = aatValue;
     		outNodeData.mMinWorstSlackRat = ratValue;
     		outNodeData.mHasTiming = true;
     	}
    }

    return true;
}

/**
 * Propagates backward with incoming edges from the vertex to set endpoint rats.
 * Does nothing and return false if either pointer is null or vertex ID is out of bounds.
 * Also ignores vertexes without pre-annotated timings in the vector.
 * @param inStaPtr target Sta
 * @param inGraphPtr target timing graph
 * @param inVertexPtr endpoint vertex
 * @param outDataVec container for output data
 * @param inEndPointGroupId ID of endpoint
 * @param inMinCondition flag of min/max condition to fill
 * @return success flag
 */
bool OpenStaServerHandler::fillSetBackpathNodesRat(
						sta::Sta* inStaPtr,
						sta::Graph* inGraphPtr,
						sta::Vertex* inVertexPtr,
						std::vector<NodeTimingData>& outDataVec,
						uint32_t inEndPointGroupId,
						bool inMinCondition) {
	if(!inStaPtr || !inGraphPtr || !inVertexPtr)
		return false;

	sta::VertexId endIdx = inVertexPtr->objectIdx();
	if(endIdx >= outDataVec.size())
		return false;

	//ignoring endpoints without the correct annotated timing
	if(!outDataVec[endIdx].mHasTiming)
		return false;

	outDataVec[endIdx].mIsEndPoint = true;
	float endMinRat = outDataVec[endIdx].mMinWorstSlackRat;
	float endMaxRat = outDataVec[endIdx].mMaxWorstSlackRat;


    std::deque<sta::Vertex*> prevVertexVec;
    std::deque<sta::Vertex*> nextVertexVec;
    prevVertexVec.push_back(inVertexPtr);

    sta::Vertex* currVertexPtr = nullptr;
    sta::VertexId currVertexId = 0;
    sta::Edge* edgePtr = nullptr;
    sta::Vertex* fromVertexPtr = nullptr;
    sta::VertexId fromVertexId = 0;

	//recursively processing all incoming edges until edge to another check isn't found
    //NOTE: also trimming out some side pathways to clock trees etc
    //because OpenSTA likes to propagate slacks to clock distribution network
    //for useful skew and the stuff
    while(!prevVertexVec.empty()) {
    	currVertexPtr = prevVertexVec.front();
    	prevVertexVec.pop_front();

    	if(!currVertexPtr)
    		continue;

    	currVertexId = currVertexPtr->objectIdx();
    	if(currVertexId >= outDataVec.size())
    		continue;

    	if(inMinCondition) {
    		outDataVec[currVertexId].mEndPointIdx = inEndPointGroupId;
        	outDataVec[currVertexId].mHasEndMinPathRat = true;
        	outDataVec[currVertexId].mMinPathRat = endMinRat;
    	} else {
    		outDataVec[currVertexId].mEndPointIdx = inEndPointGroupId;
        	outDataVec[currVertexId].mHasEndMaxPathRat = true;
        	outDataVec[currVertexId].mMaxPathRat = endMaxRat;
    	}

    	outDataVec[currVertexId].mEndPointIdx = inEndPointGroupId;

	    const sta::Pin *pin = currVertexPtr->pin();
	    std::string pinName = inStaPtr->network()->name(pin);
		//std::cout << "\t" << /*stepCount <<*/ "'" << inStaPtr->network()->name(currVertexPtr->pin()) << "'" << std::endl;

		//collecting the next vertexes with the fanin edge visitor
		sta::VertexInEdgeIterator faninEdgeIt(currVertexPtr, inGraphPtr);
		while(faninEdgeIt.hasNext()) {
			edgePtr = faninEdgeIt.next();
			fromVertexPtr = edgePtr->from(inGraphPtr);
			if(!fromVertexPtr)
				continue;

	    	fromVertexId = fromVertexPtr->objectIdx();
	    	if(fromVertexId >= outDataVec.size())
	    		continue;

			bool toSkip = !fromVertexPtr->hasRequireds() ||
					fromVertexPtr->hasChecks() ||
					fromVertexPtr->hasDownstreamClkPin() ||
					fromVertexPtr->isCheckClk() ||
					fromVertexPtr->isConstant() ||
					fromVertexPtr->isConstrained() ||
					fromVertexPtr->isDisabledConstraint() ||
					fromVertexPtr->isGatedClkEnable() ||
					fromVertexPtr->isRegClk();
			if(toSkip)
				continue;

			//entries in queue must consider RATs and AATs of previous node
			//not to propagate it on the sideways
			//separately check for min/max constraints that slack(curr) < slack(from)
			//if it is, then the from-node drives another path that is actually a critical one
			//NOTE: ignoring any additional conditions that STA uses when propagating RATs
			float currSlack = getSlack(outDataVec[currVertexId], inMinCondition);
			float fromSlack = getSlack(outDataVec[fromVertexId], inMinCondition);

			if(currSlack <= fromSlack + std::numeric_limits<float>::epsilon()) {
				//std::cout << "\t\t" << "adding from '" << inStaPtr->network()->name(fromVertexPtr->pin())
				//		<< "' currSlack=" << currSlack << " fromSlack=" << fromSlack << std::endl;
				nextVertexVec.push_back(fromVertexPtr);
			}
		}

    	//setting next wave of vertexes if current is finished
    	if(prevVertexVec.empty()) {
    		//stepCount++;
    		prevVertexVec = std::move(nextVertexVec);
    		nextVertexVec.clear(); //or isn't necessary at all?
    	}
    }

    return true;
}

/**
 * Calculates node slack depending on min/max constraint flag.
 * On max returns RAT-AAT, on min AAT-RAT.
 * Doesn't check flag that timing was set.
 * @param inData timing data
 * @param inMinCondition min condition flag
 * @return slack value
 */
float OpenStaServerHandler::getSlack(
						const NodeTimingData& inData,
						bool inMinCondition) {
	if(inMinCondition)
		return inData.mMinWorstSlackAat - inData.mMinWorstSlackRat;

	return inData.mMaxWorstSlackRat - inData.mMaxWorstSlackAat;
}



/**
 * Sets arc delays in graph.
 * Does nothing and returns false if top isntance, graph or corner is null.
 * Returns false if network isn't linked.
 * Updates graph and arc delays before assigning them.
 * Ignores invalid edge IDs.
 * Sets delay for all arcs of the edges.
 */
bool OpenStaServerHandler::execute(
						const CommandSetArcsDelays& inCommand) {
	setExecMessage("");
	if(!checkDesignLoaded())
		return false;

	sta::Corner* cornerPtr = sta::Sta::sta()->cmdCorner();
	sta::Graph* graphPtr = sta::Sta::sta()->ensureGraph();
	if(!graphPtr) {
		setExecMessage("no timing graph");
		return false;
	}

	//selecting minmax
	sta::MinMaxAll* minMaxPtr = sta::MinMaxAll::all();
	if(inCommand.mMax && !inCommand.mMin)
		minMaxPtr = sta::MinMaxAll::max();
	if(!inCommand.mMax && inCommand.mMin)
		minMaxPtr = sta::MinMaxAll::min();

	//manually setting edge delays
	sta::Edge* edgePtr = nullptr;
	for(size_t idx = 0; idx < inCommand.mDelayValuesVec.size() &&
			idx < inCommand.mEdgeIdsVec.size(); idx++) {
		edgePtr = graphPtr->edge(inCommand.mEdgeIdsVec[idx]);
		if(!edgePtr) {
			//std::cout << "no edge " << inCommand.mEdgeIdsVec[idx] << std::endl;
			continue;
		}

//		std::string fromPin;
//		std::string toPin;
//
//		sta::Vertex* fromPtr = edgePtr->from(graphPtr);
//		if(fromPtr && fromPtr->pin()) {
//			fromPin = network->name(fromPtr->pin());
//		} else {
//			std::cout << "no_from ";
//		}
//
//		sta::Vertex* toPtr = edgePtr->to(graphPtr);
//		if(toPtr && toPtr->pin()) {
//			toPin = network->name(toPtr->pin());
//		} else {
//			std::cout << "no_to ";
//		}
//
//		std::cout << "setting delay for edge " << edgePtr->objectIdx()
//				<< ": " << fromPin << " -> " << toPin << std::endl;

		//setting delay for all arcs
		for(auto* arcPtr : edgePtr->timingArcSet()->arcs()) {
//			std::cout << "\ttargeting arc" << std::endl;
			sta::Sta::sta()->setArcDelay(
					edgePtr, arcPtr, cornerPtr, minMaxPtr,
					inCommand.mDelayValuesVec[idx]);
		}
	}

	return true;
}

/**
 * Connects pin and net.
 * If pin is already connected, then disconnects it first.
 * Returns false if sta or design aren't ready.
 * Returns false if fails to find pin or net.
 */
bool OpenStaServerHandler::execute(
		const CommandConnectContextPinNet& inCommand) {
	setExecMessage("");

	return connDisPinNet(
			inCommand.mInstContextVec, inCommand.mNetName,
			inCommand.mInstName, inCommand.mPinName, true);
}

/**
 * Disconnects pin from nets.
 * Returns false if sta or design aren't ready.
 * Returns false if fails to find pin or net.
 */
bool OpenStaServerHandler::execute(
		const CommandDisconnectContextPinNet& inCommand) {
	setExecMessage("");

	return connDisPinNet(
			inCommand.mInstContextVec, inCommand.mNetName,
			inCommand.mInstName, inCommand.mPinName, false);
}


/**
 * Reads SPEF file.
 * Returns false if STA isn't ready or read failed.
 * Sets report as exec message on fail.
 */
bool OpenStaServerHandler::execute(
		const CommandReadSpefFile& inCommand) {
	setExecMessage("");
	if(!checkDesignLoaded())
		return false;

	sta::Network *network = sta::Sta::sta()->cmdNetwork();
	sta::Instance* topInstPtr = network->topInstance();
	sta::Corner* cornerPtr = sta::Sta::sta()->cmdCorner();


	if(sta::Sta::sta()->report())
		sta::Sta::sta()->report()->redirectStringBegin();

	bool isOk = sta::Sta::sta()->readSpef(
			inCommand.mStr.c_str(),
			topInstPtr, cornerPtr,
			sta::MinMaxAll::all(),
			false, false, false, 1.0,
			sta::ReducedParasiticType::none,
			false, false);

	if(sta::Sta::sta()->report())
		setExecMessage(sta::Sta::sta()->report()->redirectStringEnd());

	return isOk;
}

/**
 * Returns false.
 */
bool OpenStaServerHandler::execute(
		const CommandReadSpefStream& inCommand) {
	setExecMessage("cannot read SPEF from string");
	return false;
}


/**
 * Sets wire capacity of group of nets.
 * Sets them for both min and max conditions.
 * Returns false if Sta or design aren't ready.
 * Refreshes timing after setting capacitances.
 * Returns false if fails to find some net by it's path
 * Does nothing and returns true if command has no target nets.
 */
bool OpenStaServerHandler::execute(
		const CommandSetGroupNetCap& inCommand) {
	setExecMessage("");
	if(inCommand.mNetAddrsVec.empty() ||
			inCommand.mValuesVec.empty())
		return true;

	if(!checkDesignLoaded())
		return false;

	sta::Sta* staPtr = sta::Sta::sta();
	sta::Network* networkPtr = staPtr->cmdNetwork();
	sta::Corner* cornerPtr = staPtr->cmdCorner();

	sta::Net* netPtr = nullptr;
	bool allOk = true;

	//for each net searching it's instance context
	//then setting capacitance
	for(size_t netIdx = 0; netIdx < inCommand.mNetAddrsVec.size() &&
			netIdx < inCommand.mValuesVec.size();
			netIdx++) {
		netPtr = findNetInContext(networkPtr,
				inCommand.mNetAddrsVec[netIdx].mInstContextVec,
				inCommand.mNetAddrsVec[netIdx].mObjName);
		//ignoring if net doesn't exist
		if(!netPtr) {
			addExecMessage("couldn't find net " +
					inCommand.mNetAddrsVec[netIdx].mObjName);
			allOk = false;
			continue;
		}

		//setting the load capacitance
		sta::Sta::sta()->setNetWireCap(
				netPtr, false, cornerPtr,
				sta::MinMaxAll::all(),
				inCommand.mValuesVec[netIdx]);
	}

	//refreshing delays and slacks
	sta::Sta::sta()->searchPreamble();
	sta::Sta::sta()->updateTiming(false);

	return allOk;
}

/**
 * Returns false.
 */
bool OpenStaServerHandler::execute(
		const CommandReadSdfFile& inCommand) {
	setExecMessage("cannot read SDF file");
	return false;
}

/**
 * Returns false.
 */
bool OpenStaServerHandler::execute(
		const CommandReadSdfStream& inCommand) {
	setExecMessage("cannot read SDF string");
	return false;
}

/**
 * Writes timings in SDF file.
 * Returns false if Sta or design isn't ready.
 */
bool OpenStaServerHandler::execute(
		const CommandWriteSdfFile& inCommand) {
	setExecMessage("");
	if(!checkStaReady())
		return false;

	sta::Graph* graphPtr = sta::Sta::sta()->ensureGraph();
	if(!graphPtr) {
		setExecMessage("no timing graph");
		return false;
	}

	sta::Search* searchPtr = sta::Sta::sta()->search();
	if(!searchPtr)
		return false;

	sta::Corner* cornerPtr = sta::Sta::sta()->cmdCorner();

	sta::Sta::sta()->searchPreamble();
	searchPtr->findAllArrivals();
	searchPtr->findRequireds();

	if(sta::Sta::sta()->report())
		sta::Sta::sta()->report()->redirectStringBegin();

	sta::Sta::sta()->writeSdf(inCommand.mStr.c_str(), cornerPtr, '/', 3, false, false, false);

	if(sta::Sta::sta()->report())
		setExecMessage(sta::Sta::sta()->report()->redirectStringEnd());

	return true;
}

/**
 * Check timing constraints and writes out report string.
 * Returns false if design wasn't loaded or there're no path ends.
 * @param inCommand command to execute
 * @param outReportStr report string to write out
 * @success flag
 */
bool OpenStaServerHandler::execute(
						const CommandReportTiming& inCommand,
						std::string& outReportStr) {
	setExecMessage("");
	if(!checkDesignLoaded())
		return false;

	sta::Corner* cornerPtr = sta::Sta::sta()->cmdCorner();
	sta::Report *report = sta::Sta::sta()->report();
	if(report)
		report->redirectStringBegin();

	//in OpenSTA cmd by default corner is "NULL" if no corner is specified
//	sta::Corner* cornerPtr = nullptr;

	sta::Sta::sta()->searchPreamble();

	sta::Search* searchPtr = sta::Sta::sta()->search();
	if(searchPtr) {
		searchPtr->findAllArrivals();
		searchPtr->findRequireds();
	}

	sta::MinMaxAll* minMaxPtr = sta::MinMaxAll::all();
	if(!inCommand.mMin && inCommand.mMax)
		minMaxPtr = sta::MinMaxAll::max();
	if(inCommand.mMin && !inCommand.mMax)
		minMaxPtr = sta::MinMaxAll::min();

	sta::PathEndSeq *ends = sta::Sta::sta()->findPathEnds(
			nullptr, nullptr, nullptr,
			inCommand.mUnconstrained, cornerPtr,
			minMaxPtr, inCommand.mGroupsNum,
			inCommand.mEndPointsNum,
			inCommand.mUniquePaths,
			-sta::INF, sta::INF,
			false, nullptr,
			true, true, true, true, true, true);


	if(!ends) {
		setExecMessage("no timing path ends");
		return false;
	}

	sta::Sta::sta()->reportPathEnds(ends);
	if(ends) {
		delete ends;
		ends = nullptr;
	}

	outReportStr = report->redirectStringEnd();

	return true;
}


/**
 * Gets source pins, waveforms, then creates clock.
 * Returns false if network isn't linked or one of sources doesn't exist.
 * @param inCommand command to execute
 * @return execution status
 */
bool OpenStaServerHandler::execute(
						const CommandCreateClock& inCommand) {
	setExecMessage("");
	if(!checkDesignLoaded())
		return false;

	sta::Network* networkPtr = sta::Sta::sta()->cmdNetwork();

	//searching and filling source pins
	sta::PinSet* pinSetPtr = new sta::PinSet();
	if(!findAddObjectsByAddr<sta::Pin, TargetSearchObjType::SearchObjTypePin>(
			networkPtr, networkPtr->topInstance(),
			inCommand.mPinPathsVec, pinSetPtr)) {
		setExecMessage("failed to find clock source");
		delete pinSetPtr;
		return false;
	}

	//creating waveform sequence
	sta::FloatSeq* wavesSeqPtr = new sta::FloatSeq();
	wavesSeqPtr->insert(wavesSeqPtr->end(),
			inCommand.mWaveformVec.begin(),
			inCommand.mWaveformVec.end());

	//get char* from string,
	//OpenSTA inside passes it as const char* through...
	char* comment = strcpy(
			new char[inCommand.mDescription.length() + 1],
			inCommand.mDescription.c_str());

	sta::Sta::sta()->makeClock(
			inCommand.mName.c_str(),
			pinSetPtr,
			inCommand.mAddFlag,
			inCommand.mPeriod,
			wavesSeqPtr, comment);

	//NOTE: comment string is only copied, not cleaned up inside the method
	delete [] comment;

	//OpenSTA deletes allocated set of pins inside clock's setter method
	//delete pinSetPtr;
	return true;
}


/**
 * Gets source pins, master clock's pin and other options, then creates generated clock.
 * Returns false if Sta isn't ready or one of sources doesn't exist.
 * Returns false if fails to find source pin if it's name isn't empty.
 * Returns false if master clock doesn't exist.
 * @param inCommand command to execute
 * @return execution status
 */
bool OpenStaServerHandler::execute(
						const CommandCreateGenClock& inCommand) {
	if(!checkDesignLoaded())
		return false;

	sta::Network* networkPtr = sta::Sta::sta()->cmdNetwork();
	bool allOk = true;

	setExecMessage("");

	//searching and filling source pins
	sta::PinSet* pinSetPtr = new sta::PinSet();
	if(!findAddObjectsByAddr<sta::Pin, TargetSearchObjType::SearchObjTypePin>(
			networkPtr, networkPtr->topInstance(),
			inCommand.mPinPathsVec, pinSetPtr)) {
		addExecMessage("failed to find clock source");
		allOk = false;
	}

	//searching master clock's pin
	sta::Pin* masterPin = nullptr;
	if(!inCommand.mMasterClockPinPath.mObjName.empty()) {
		masterPin = findObjInContext<sta::Pin, TargetSearchObjType::SearchObjTypePin>(
				networkPtr, networkPtr->topInstance(),
				inCommand.mMasterClockPinPath.mInstContextVec,
				inCommand.mMasterClockPinPath.mObjName);
		if(!masterPin) {
			addExecMessage("no master-clock pin " + inCommand.mMasterClockPinPath.mObjName);
			allOk = false;
		}
	}

	//searching master clock by it's name
	sta::Clock* masterClockPtr = sta::Sta::sta()->sdc()->
			findClock(inCommand.mMasterClockName.c_str());
	if(!masterClockPtr) {
		addExecMessage("no master-clock " + inCommand.mMasterClockName);
		allOk = false;
	}

	//quote: edges size must be 3.
	sta::IntSeq* edgesPtr = nullptr;
	sta::FloatSeq* edgeShiftsPtr = nullptr;

	//using input edges if they aren't empty
	//also they must be of correct size
	if(!inCommand.mEdgeShiftsVec.empty() &&
			!inCommand.mEdgesVec.empty()) {
		if(inCommand.mEdgeShiftsVec.size() != inCommand.mEdgesVec.size() ||
				inCommand.mEdgeShiftsVec.size() != 3) {
			addExecMessage("on use edges and shifts must contain 3 entries");
			allOk = false;
		}
		edgesPtr = new sta::IntSeq;
		edgesPtr->insert(edgesPtr->end(),
				inCommand.mEdgesVec.begin(),
				inCommand.mEdgesVec.end());

		edgeShiftsPtr = new sta::FloatSeq;
		edgeShiftsPtr->insert(edgeShiftsPtr->end(),
				inCommand.mEdgeShiftsVec.begin(),
				inCommand.mEdgeShiftsVec.end());
	}

	char* comment = strcpy(
			new char[inCommand.mDescription.length() + 1],
			inCommand.mDescription.c_str());

	if(allOk)
		sta::Sta::sta()->makeGeneratedClock(
			inCommand.mName.c_str(),
			pinSetPtr,
			inCommand.mAddFlag,
			masterPin, masterClockPtr,
			nullptr, nullptr,
			inCommand.mDivideFactor,
			inCommand.mMultiplyFactor,
			inCommand.mDutyCycle,
			inCommand.mInvert,
			false,
			edgesPtr, edgeShiftsPtr,
			comment);

	//NOTE: pins, edges and edge_shifts are cleaned up inside the method
//	delete pinSetPtr;
//	if(edgesPtr)
//		delete edgesPtr;
//	if(edgeShiftsPtr)
//		delete edgeShiftsPtr;

	//NOTE: comment string is only copied, not cleaned up inside the method
	delete [] comment;

	return allOk;
}


/**
 * Makes clock group, then adds clock sets one by one.
 * Ignores empty clock sets or those entirely of inexistent clocks.
 * Ignores unexistent clocks.
 * Returns false if Sta isn't ready.
 * Returns false if fails to create clocks group.
 * @param inCommand command to execute
 * @return execution status
 */
bool OpenStaServerHandler::execute(
						const CommandSetClockGroups& inCommand) {
	setExecMessage("");
	if(!checkDesignLoaded())
		return false;

	sta::ClockGroups* groupsPtr =
			sta::Sta::sta()->makeClockGroups(
					inCommand.mName.c_str(),
					inCommand.mLogicalExclusive,
					inCommand.mPhysicalExclusive,
					inCommand.mAsynchronous,
					inCommand.mAllowPaths,
					inCommand.mDescription.c_str());


	if(!groupsPtr) {
		setExecMessage("failed to create clocks group");
		return false;
	}

	sta::ClockSet* clkSetPtr = nullptr;
	sta::Clock* clockPtr = nullptr;

	setExecMessage("");

	//processing each clocks set and adding in to the group
	for(const std::vector<std::string>& clkNamesVec : inCommand.mClockGroupsVec) {
		//ignoring empty ones
		if(clkNamesVec.empty())
			continue;

		clkSetPtr = new sta::ClockSet();
		//locating each clock and adding it
		for(const std::string& clkName : clkNamesVec) {
			clockPtr = sta::Sta::sta()->sdc()->findClock(clkName.c_str());
			if(!clockPtr) {
				addExecMessage("no clock " + clkName);
				continue;
			}

			clkSetPtr->insert(clockPtr);
		}

		//ignoring empty sets
		if(clkSetPtr->empty()) {
			delete clkSetPtr;
			continue;
		}

		sta::Sta::sta()->makeClockGroup(groupsPtr, clkSetPtr);
	}

	return true;
}


/**
 * Sets clock latency or insertion.
 * Returns false if Sta isn't ready.
 * Returns false if fails to find both target clock and pin.
 * Sets rise/fall to both if they are true/true or false/false.
 * Sets min/max to both if they are true/true or false/false.
 * Sets early/late to both if they are true/true or false/false.
 * For clock latency ignores early/late flags.
 * If source flag is set, then adds clock insertion.
 * @param inCommand command to execute
 * @return execution status
 */
bool OpenStaServerHandler::execute(
						const CommandSetClockLatency& inCommand) {
	if(!checkDesignLoaded())
		return false;

	sta::Network* networkPtr = sta::Sta::sta()->cmdNetwork();

	sta::Clock* clockPtr = sta::Sta::sta()->sdc()->
			findClock(inCommand.mClockName.c_str());

	sta::Pin* pinPtr = findObjInContext<sta::Pin, TargetSearchObjType::SearchObjTypePin>(
			networkPtr, networkPtr->topInstance(),
			inCommand.mPinPath.mInstContextVec,
			inCommand.mPinPath.mObjName);

	setExecMessage("");
	if(!clockPtr)
		addExecMessage("no target clock " + inCommand.mClockName);
	if(!pinPtr)
		addExecMessage("no source clock pin " + inCommand.mPinPath.mObjName);

	//clock or pin must exist
	//latency can be pin/clock/pin+clock
	if(!clockPtr && !pinPtr)
		return false;

	//clock latency based on min/max, early/late are ignored
	sta::MinMaxAll* earlyLatePtr = getEarlyLateAll(inCommand.mEarly, inCommand.mLate);
	sta::MinMaxAll* minMaxPtr = getMinMaxAll(inCommand.mMin, inCommand.mMax);
	sta::RiseFallBoth* rfPtr = getRiseFallBoth(inCommand.mRise, inCommand.mFall);

	if(inCommand.mSource) {
		sta::Sta::sta()->setClockLatency(
				clockPtr, pinPtr, rfPtr, minMaxPtr, inCommand.mValue);
	} else {
		sta::Sta::sta()->setClockInsertion(
				clockPtr, pinPtr, rfPtr, minMaxPtr, earlyLatePtr, inCommand.mValue);
	}

	return true;
}

/**
 * Sets clock uncertainty between two clocks.
 * Returns false if Sta isn't ready.
 * Returns false if one of clocks doesn't exist.
 * Sets rise/fall to both if they are true/true or false/false.
 * Sets setup/hold to both if they are true/true or false/false.
 * @param inCommand command to execute
 * @return execution status
 */
bool OpenStaServerHandler::execute(
						const CommandSetInterClockUncertainty& inCommand) {
	if(!checkDesignLoaded())
		return false;

	sta::Clock* fromClockPtr = sta::Sta::sta()->sdc()->
			findClock(inCommand.mFromClockName.c_str());
	sta::Clock* toClockPtr = sta::Sta::sta()->sdc()->
			findClock(inCommand.mToClockName.c_str());

	setExecMessage("");
	if(!fromClockPtr)
		addExecMessage("couldn't find from-clock " + inCommand.mFromClockName);
	if(!fromClockPtr)
		addExecMessage("couldn't find to-clock " + inCommand.mToClockName);

	//both clocks must exist
	if(!fromClockPtr || !toClockPtr)
		return false;

	sta::RiseFallBoth* fromRfPtr = getRiseFallBoth(
			inCommand.mFromRise, inCommand.mFromFall);
	sta::RiseFallBoth* toRfPtr = getRiseFallBoth(
			inCommand.mToRise, inCommand.mToFall);
	sta::SetupHoldAll* shPtr = getSetupHoldAll(
			inCommand.mSetup, inCommand.mHold);

	sta::Sta::sta()->setClockUncertainty(
			fromClockPtr, fromRfPtr,
			toClockPtr, toRfPtr,
			shPtr, inCommand.mValue);

	return true;
}

/**
 * Sets uncertainty of single clock.
 * Returns false if Sta isn't ready.
 * Returns false if clock doesn't exist.
 * Sets setup/hold to both if they are true/true or false/false.
 * @param inCommand command to execute
 * @return execution status
 */
bool OpenStaServerHandler::execute(
						const CommandSetSingleClockUncertainty& inCommand) {
	if(!checkDesignLoaded())
		return false;

	setExecMessage("");
	sta::Clock* clockPtr = sta::Sta::sta()->sdc()->
			findClock(inCommand.mClockName.c_str());

	//clock must exist
	if(!clockPtr) {
		setExecMessage("couldn't find clock " + inCommand.mClockName);
		return false;
	}

	sta::SetupHoldAll* shPtr = getSetupHoldAll(
			inCommand.mSetup, inCommand.mHold);

	sta::Sta::sta()->setClockUncertainty(clockPtr, shPtr, inCommand.mValue);
	return true;
}

/**
 * Sets uncertainty of single pin.
 * Returns false if Sta isn't ready.
 * Returns false if pin doesn't exist.
 * Sets setup/hold to both if they are true/true or false/false.
 * @param inCommand command to execute
 * @return execution status
 */
bool OpenStaServerHandler::execute(
		const CommandSetSinglePinUncertainty& inCommand) {
	if(!checkDesignLoaded())
		return false;

	setExecMessage("");
	sta::Network* networkPtr = sta::Sta::sta()->cmdNetwork();

	sta::Pin* pinPtr = findObjInContext<sta::Pin, TargetSearchObjType::SearchObjTypePin>(
			networkPtr, networkPtr->topInstance(),
			inCommand.mPinPath.mInstContextVec,
			inCommand.mPinPath.mObjName);
	//pin must exist
	if(!pinPtr) {
		setExecMessage("couldn't find target uncertainty pin " +
				inCommand.mPinPath.mObjName);
		return false;
	}
	sta::SetupHoldAll* shPtr = getSetupHoldAll(
			inCommand.mSetup, inCommand.mHold);

	sta::Sta::sta()->setClockUncertainty(pinPtr, shPtr, inCommand.mValue);
	return true;
}

/**
 * Sets input/output delay respective (or not) to clock.
 * Returns false if Sta isn't ready.
 * Returns false if target pin doesn't exist.
 * Returns false if source pin of the clock is the same as target pin.
 * Sets setup/hold to both if they are true/true or false/false.
 * @param inCommand command to execute
 * @return execution status
 */
bool OpenStaServerHandler::execute(
						const CommandSetPortDelay& inCommand) {
	if(!checkDesignLoaded())
		return false;

	setExecMessage("");
	sta::Network* networkPtr = sta::Sta::sta()->cmdNetwork();

	//target pin must exist
	sta::Pin* targetPinPtr = findObjInContext<sta::Pin, TargetSearchObjType::SearchObjTypePin>(
			networkPtr, networkPtr->topInstance(),
			inCommand.mTargetPortPin.mInstContextVec,
			inCommand.mTargetPortPin.mObjName);
	if(!targetPinPtr) {
		setExecMessage("couldn't find target delay port " +
				inCommand.mTargetPortPin.mObjName);
		return false;
	}

	sta::Clock* clockPtr = nullptr;
	if(!inCommand.mClockName.empty())
		clockPtr = sta::Sta::sta()->sdc()->
				findClock(inCommand.mClockName.c_str());

	sta::RiseFall* clkRfPtr = sta::RiseFall::rise();
	if(inCommand.mClockFall)
		clkRfPtr = sta::RiseFall::fall();

	sta::Pin* clockPinPtr = nullptr;
	if(!inCommand.mClockPinPath.mObjName.empty() && clockPtr)
		clockPinPtr = findObjInContext<sta::Pin, TargetSearchObjType::SearchObjTypePin>(
					networkPtr, networkPtr->topInstance(),
					inCommand.mClockPinPath.mInstContextVec,
					inCommand.mClockPinPath.mObjName);

	//target pin must not be the same as the clock's pin
	if(clockPinPtr == targetPinPtr) {
		setExecMessage("delay port is the same as clock source");
		return false;
	}

	sta::MinMaxAll* minMaxPtr = getMinMaxAll(inCommand.mDelayMin, inCommand.mDelayMax);
	sta::RiseFallBoth* rfPtr = getRiseFallBoth(inCommand.mDelayRise, inCommand.mDelayFall);


	//calling method for corrsponding signal direction
	if(inCommand.mIsInput) {
		sta::Sta::sta()->setInputDelay(
			  targetPinPtr, rfPtr, clockPtr, clkRfPtr, clockPinPtr,
			  inCommand.mSourceLatencyInc, inCommand.mNetworkLatencyInc,
			  minMaxPtr, inCommand.mAdd, inCommand.mDelay);
	} else {
		sta::Sta::sta()->setOutputDelay(
			  targetPinPtr, rfPtr, clockPtr, clkRfPtr, clockPinPtr,
			  inCommand.mSourceLatencyInc, inCommand.mNetworkLatencyInc,
			  minMaxPtr, inCommand.mAdd, inCommand.mDelay);
	}

	return true;
}

/**
 * Sets input port transition.
 * Returns false if Sta isn't ready.
 * Returns false if target port doesn't exist.
 * Returns false if slew value isn't positive.
 * @param inCommand command to execute
 * @return execution status
 */
bool OpenStaServerHandler::execute(
						const CommandSetInPortTransition& inCommand) {
	if(!checkDesignLoaded())
		return false;

	setExecMessage("");
	sta::Network* networkPtr = sta::Sta::sta()->cmdNetwork();

	//target port must exist
	sta::Cell* masterCellPtr = networkPtr->cell(networkPtr->topInstance());
	if(!masterCellPtr) {
		setExecMessage("couldn't find top-block");
		return false;
	}

	sta::Port* targetPortPtr = networkPtr->findPort(
			masterCellPtr, inCommand.mTargetPortPin.mObjName.c_str());
	if(!targetPortPtr) {
		setExecMessage("couldn't find transition port " +
				inCommand.mTargetPortPin.mObjName);
		return false;
	}

	sta::MinMaxAll* minMaxPtr = getMinMaxAll(inCommand.mDelayMin, inCommand.mDelayMax);
	sta::RiseFallBoth* rfPtr = getRiseFallBoth(inCommand.mDelayRise, inCommand.mDelayFall);

	//slew value must be positive
	if(inCommand.mValue <= 0) {
		setExecMessage("port transition value must be positive");
		return false;
	}

	sta::Sta::sta()->setInputSlew(targetPortPtr, rfPtr, minMaxPtr, inCommand.mValue);
	return true;
}


/**
 * Sets false path.
 * Returns false if Sta isn't ready.
 * Returns false if no path points were specified.
 * Treats all thru entries as a single one.
 * @param inCommand command to execute
 * @return execution status
 */
bool OpenStaServerHandler::execute(
						const CommandSetFalsePath& inCommand) {
	if(!checkDesignLoaded())
		return false;

	setExecMessage("");
	sta::Network* networkPtr = sta::Sta::sta()->cmdNetwork();

	//from/through/to pointer can be null when there're no corresponding entries
	sta::ExceptionFrom* fromExceptPtr =
			createPathExceptionFrom(
					sta::Sta::sta()->sdc(),
					networkPtr,
					networkPtr->topInstance(),
					inCommand);

	sta::ExceptionThruSeq* thruExceptVecPtr =
			createPathExceptionThrus(
					networkPtr,
					networkPtr->topInstance(),
					inCommand);

	sta::ExceptionTo* toExceptPtr =
			createPathExceptionTo(
					sta::Sta::sta()->sdc(),
					networkPtr,
					networkPtr->topInstance(),
					inCommand);

	//at least one point must exist
	if(!fromExceptPtr && !thruExceptVecPtr && !toExceptPtr) {
		setExecMessage("at least one from/thru/to point must exist");
		return false;
	}

	sta::SetupHoldAll* shPtr = getSetupHoldAll(
			inCommand.mSetup, inCommand.mHold);

	sta::Sta::sta()->makeFalsePath(
			fromExceptPtr, thruExceptVecPtr, toExceptPtr,
			shPtr, inCommand.mComment.c_str());

	return true;
}


/**
 * Sets min or max delay constraint for paths.
 * Returns false if Sta isn't ready.
 * Returns false if no path points were specified.
 * Treats all thru entries as a single one.
 * Ignore clock latency is set to false.
 * @param inCommand command to execute
 * @return execution status
 */
bool OpenStaServerHandler::execute(
		const CommandSetMinMaxDelay& inCommand) {
	if(!checkDesignLoaded())
		return false;

	setExecMessage("");
	sta::Network* networkPtr = sta::Sta::sta()->cmdNetwork();

	//from/through/to pointer can be null when there're no corresponding entries
	sta::ExceptionFrom* fromExceptPtr =
			createPathExceptionFrom(
					sta::Sta::sta()->sdc(),
					networkPtr,
					networkPtr->topInstance(),
					inCommand);

	sta::ExceptionThruSeq* thruExceptVecPtr =
			createPathExceptionThrus(
					networkPtr,
					networkPtr->topInstance(),
					inCommand);

	sta::ExceptionTo* toExceptPtr =
			createPathExceptionTo(
					sta::Sta::sta()->sdc(),
					networkPtr,
					networkPtr->topInstance(),
					inCommand);

	//at least one point must exist
	if(!fromExceptPtr && !thruExceptVecPtr && !toExceptPtr) {
		setExecMessage("at least one from/thru/to point must exist");
		return false;
	}

	sta::MinMax* minMaxPtr = sta::MinMax::max();
	if(inCommand.mMinDelayFlag)
		minMaxPtr = sta::MinMax::min();

	sta::Sta::sta()->makePathDelay(
			fromExceptPtr, thruExceptVecPtr, toExceptPtr,
			minMaxPtr, false, inCommand.mValue,
			inCommand.mComment.c_str());

	return true;
}

/**
 * Sets multicycle delay coefficient for paths.
 * Returns false if Sta isn't ready.
 * Returns false if no path points were specified.
 * Treats all thru entries as a single one.
 * By default uses end clock, if start flag is set, only then uses start clock.
 * If start and end flags are false, then uses end clock.
 * @param inCommand command to execute
 * @return execution status
 */
bool OpenStaServerHandler::execute(
		const CommandSetMulticyclePath& inCommand) {
	if(!checkDesignLoaded())
		return false;

	setExecMessage("");
	sta::Network* networkPtr = sta::Sta::sta()->cmdNetwork();

	//from/through/to pointer can be null when there're no corresponding entries
	sta::ExceptionFrom* fromExceptPtr =
			createPathExceptionFrom(
					sta::Sta::sta()->sdc(),
					networkPtr,
					networkPtr->topInstance(),
					inCommand);

	sta::ExceptionThruSeq* thruExceptVecPtr =
			createPathExceptionThrus(
					networkPtr,
					networkPtr->topInstance(),
					inCommand);

	sta::ExceptionTo* toExceptPtr =
			createPathExceptionTo(
					sta::Sta::sta()->sdc(),
					networkPtr,
					networkPtr->topInstance(),
					inCommand);

	//at least one point must exist
	if(!fromExceptPtr && !thruExceptVecPtr && !toExceptPtr) {
		setExecMessage("at least one from/thru/to point must exist");
		return false;
	}

	sta::SetupHoldAll* shPtr = getSetupHoldAll(
			inCommand.mSetup, inCommand.mHold);

	//by default
	bool useEndClk = true;
	if(inCommand.mStart)
		useEndClk = false;

	sta::Sta::sta()->makeMulticyclePath(
			fromExceptPtr, thruExceptVecPtr, toExceptPtr,
			shPtr, useEndClk, inCommand.mValue,
			inCommand.mComment.c_str());

	return true;
}

/**
 * Disables pin's timing arcs.
 * Returns false if Sta isn't ready.
 * Returns false if pin path is empty or pin wasn't found.
 * @param inCommand command to execute
 * @return execution status
 */
bool OpenStaServerHandler::execute(
		const CommandDisableSinglePinTiming& inCommand) {
	if(!checkDesignLoaded())
		return false;

	setExecMessage("");
	sta::Network* networkPtr = sta::Sta::sta()->cmdNetwork();

	if(inCommand.mPinPath.mObjName.empty()) {
		setExecMessage("target pin to disable wasn't specified");
		return false;
	}

	sta::Pin* targetPin = findObjInContext<sta::Pin, TargetSearchObjType::SearchObjTypePin>(
				networkPtr, networkPtr->topInstance(),
				inCommand.mPinPath.mInstContextVec,
				inCommand.mPinPath.mObjName);
	if(!targetPin) {
		setExecMessage("failed to find disable pin " +
				inCommand.mPinPath.mObjName);
		return false;
	}
	sta::Sta::sta()->disable(targetPin);
	return true;
}

/**
 * Disables timing arcs between two pins of the instance.
 * Returns false if Sta isn't ready.
 * Returns false if inst path is empty.
 * Returns false if inst or it's cell wasn't found.
 * Uses nullptr if one of pin names is empty.
 * If pin name isn't empty and wasn't found, then returns false.
 * @param inCommand command to execute
 * @return execution status
 */
bool OpenStaServerHandler::execute(
		const CommandDisableInstTiming& inCommand) {
	if(!checkDesignLoaded())
		return false;

	setExecMessage("");
	sta::Network* networkPtr = sta::Sta::sta()->cmdNetwork();

	//inst and it's path must exist
	if(inCommand.mInstContextVec.empty())
		return false;

	sta::Instance* instPtr = findContextInstance(
			networkPtr, networkPtr->topInstance(), inCommand.mInstContextVec);
	if(!instPtr) {
		setExecMessage("failed to find disable instance");
		return false;
	}

	sta::Cell* cellPtr = networkPtr->cell(instPtr);
	if(!cellPtr) {
		setExecMessage("failed to find master block of disable instance");
		return false;
	}

	//finding ports in cell and converting them in liberty ports

	sta::LibertyPort* fromPortPtr = nullptr;
	if(!inCommand.mFromPinName.empty()) {
		sta::Port* portPtr = networkPtr->findPort(
				cellPtr, inCommand.mFromPinName.c_str());
		if(portPtr)
			fromPortPtr = networkPtr->libertyPort(portPtr);

		if(!fromPortPtr) {
			setExecMessage("failed to edge source port of disable instance " +
					inCommand.mFromPinName);
			return false;
		}
	}

	sta::LibertyPort* toPortPtr = nullptr;
	if(!inCommand.mToPinName.empty()) {
		sta::Port* portPtr = networkPtr->findPort(
				cellPtr, inCommand.mToPinName.c_str());
		if(portPtr)
			toPortPtr = networkPtr->libertyPort(portPtr);

		if(!toPortPtr) {
			setExecMessage("failed to edge target port of disable instance " +
					inCommand.mFromPinName);
			return false;
		}
	}

	sta::Sta::sta()->disable(instPtr, fromPortPtr, toPortPtr);
	return true;
}

/**
 * Sets global derate factor.
 * Returns false if Sta isn't ready.
 * If no derate type is set, then uses cell_delay and net_delay.
 * Otherwise separately sets derate for each derate type.
 * If no path type is set, then uses data and clock.
 * Otherwise separates sets derate for each path type.
 * If early and late both are set, then sets derate for each of them separately.
 * If none if them is set, then sets derate for late timings.
 * @param inCommand command to execute
 * @return execution status
 */
bool OpenStaServerHandler::execute(
						const CommandSetGlobalTimingDerate& inCommand) {
	setExecMessage("");
	if(!checkStaReady())
		return false;

	std::vector<sta::TimingDerateType> derateTypesVec;
	//setting net and cell delays if no flags were specified
	if(!inCommand.mCellCheck &&
			!inCommand.mCellDelay &&
			!inCommand.mNetDelay) {
		derateTypesVec.push_back(sta::TimingDerateType::cell_delay);
		derateTypesVec.push_back(sta::TimingDerateType::net_delay);
	}

	if(inCommand.mCellCheck)
		derateTypesVec.push_back(sta::TimingDerateType::cell_check);
	if(inCommand.mCellDelay)
		derateTypesVec.push_back(sta::TimingDerateType::cell_delay);
	if(inCommand.mNetDelay)
		derateTypesVec.push_back(sta::TimingDerateType::net_delay);

	//selecting clock/data, both if neigher is true
	std::vector<sta::PathClkOrData> pathTypesVec;
	if(!inCommand.mData && !inCommand.mClock)
		pathTypesVec = {sta::PathClkOrData::clk, sta::PathClkOrData::data};
	if(inCommand.mData)
		pathTypesVec.push_back(sta::PathClkOrData::data);
	if(inCommand.mClock)
		pathTypesVec.push_back(sta::PathClkOrData::clk);

	sta::RiseFallBoth* rfPtr = getRiseFallBoth(
			inCommand.mRise, inCommand.mFall);

	//if both early and late are specified, then adding both
	//if none, then late
	std::vector<sta::EarlyLate*> earlyLateVec;
	if(inCommand.mEarly)
		earlyLateVec.push_back(sta::EarlyLate::early());
	if(inCommand.mLate)
		earlyLateVec.push_back(sta::EarlyLate::late());
	if(earlyLateVec.empty())
		earlyLateVec.push_back(sta::EarlyLate::late());

	//adding derate for each type
	for(sta::EarlyLate* earlyLatePtr : earlyLateVec) {
		for(sta::PathClkOrData pathType : pathTypesVec) {
			for(sta::TimingDerateType derateType : derateTypesVec) {
				sta::Sta::sta()->setTimingDerate(derateType, pathType, rfPtr, earlyLatePtr, inCommand.mValue);
			}
		}
	}

	return true;
}


/**
 * Sets message to return after command execution.
 * @param inMessage execution message
 */
void OpenStaServerHandler::setExecMessage(
						const std::string& inMessage) {
	mExecResultMessage = inMessage;
}


/**
 * Appends message to existing execution message.
 * Adds endline if existing message isn't empty.
 * @param inMessage message to append
 */
void OpenStaServerHandler::addExecMessage(
						const std::string& inMessage) {
	if(!mExecResultMessage.empty())
		mExecResultMessage += "\n";

	mExecResultMessage += inMessage;
}


/**
 * Checks that sta class was set up and has network, corner etc.
 * Set up execution message and reutrns false is something is wrong.
 * @return setup state
 */
bool OpenStaServerHandler::checkStaReady() {

	if(!sta::Sta::sta()) {
		setExecMessage("no Sta class");
		return false;
	}

	if(!sta::Sta::sta()->sdc()) {
		setExecMessage("no Sdc class");
		return false;
	}
	if(!sta::Sta::sta()->cmdNetwork()) {
		setExecMessage("no cmd network class");
		return false;
	}
	if(!sta::Sta::sta()->search()) {
		setExecMessage("no Search class");
		return false;
	}
	if(!sta::Sta::sta()->cmdCorner()) {
		setExecMessage("no cmd corner class");
		return false;
	}

	return true;
}

/**
 * Checks that sta is set up and has top-level instance.
 * \link checkStaReady
 * @return setup state
 */
bool OpenStaServerHandler::checkDesignLoaded() {
	if(!checkStaReady())
		return false;

	if(!sta::Sta::sta()->cmdNetwork()->isLinked()) {
		setExecMessage("network isn't linked");
		return false;
	}
	if(!sta::Sta::sta()->cmdNetwork()->topInstance()) {
		setExecMessage("network has no top-level instance");
		return false;
	}

	return true;
}

/**
 * Processes block data in vector to find first one with top-level flag.
 * Returns max(uint32_t) if fails to find index of top-level block.
 * @param inBlocksDataVec data of netlist blocks
 * @return index of top-block or max(uint32_t)
 */
uint32_t OpenStaServerHandler::findTopBlockIdx(
						const std::vector<BlockData>& inBlocksDataVec) {
	for(uint32_t idx = 0; idx < inBlocksDataVec.size(); idx++) {
		if(inBlocksDataVec[idx].mTopFlag)
			return idx;
	}

	return std::numeric_limits<uint32_t>::max();
}

/**
 * Finds library in network, creates it if it doesn't exist.
 * Does nothing and returns false if network pointer is null.
 * @param inNetworkPtr target network
 * @return library pointer or nullptr
 */
sta::Library* OpenStaServerHandler::getCreateNetlistLib(
						sta::NetworkReader* inNetworkPtr) {
	if(!inNetworkPtr)
		return nullptr;

	sta::Library* libPtr = inNetworkPtr->findLibrary(mcNetlistLibName.c_str());
	if(!libPtr)
		libPtr = inNetworkPtr->makeLibrary(mcNetlistLibName.c_str(), nullptr);

	return libPtr;
}


/**
 * Searches for existing block declaration in OpenSTA libraries.
 * If it wasn't found, then creates new one in provided library.
 * Returns nullptr if either of input pointers is null.
 * Uses \link createBlockDeclaration to create cell.
 * @param inNetworkPtr target network
 * @param inLibraryPtr target library
 * @param inBlockData data of targe block
 * @return cell pointer or nullptr
 */
sta::Cell* OpenStaServerHandler::findCreateBlockDeclaration(
						sta::NetworkReader* inNetworkPtr,
						sta::Library* inLibraryPtr,
						const BlockData& inBlockData) {
	if(!inNetworkPtr || !inLibraryPtr)
		return nullptr;

	sta::Cell* cellPtr = inNetworkPtr->findAnyCell(inBlockData.mName.c_str());

	//if block cell is leaf, then trying to find liberty cell
	if(inBlockData.mLeafFlag && cellPtr) {
		sta::LibertyCell* libCellPtr = inNetworkPtr->libertyCell(cellPtr);
		if (libCellPtr)
			cellPtr = inNetworkPtr->cell(libCellPtr);
	}

	if(!cellPtr)
		cellPtr = createBlockDeclaration(
				inNetworkPtr, inLibraryPtr, inBlockData);

	return cellPtr;
}

/**
 * Creates cell declaration in library.
 * First creates cell itself, then add ports.
 * Does nothing and returns false if library or network are null.
 * Returns false if fails to create cell.
 * @param inNetworkPtr target network
 * @param inLibraryPtr target library
 * @param inBlockData data of targe block
 * @return cell pointer or nullptr
 */
sta::Cell* OpenStaServerHandler::createBlockDeclaration(
						sta::NetworkReader* inNetworkPtr,
						sta::Library* inLibraryPtr,
						const BlockData& inBlockData) {
	if(!inNetworkPtr || !inLibraryPtr)
		return nullptr;

	//creating the cell itself
	sta::Cell* cellPtr = inNetworkPtr->makeCell(
			inLibraryPtr, inBlockData.mName.c_str(), false, "");
	if(!cellPtr)
		return nullptr;

	//and creating it's ports
	sta::PortDirection* portDirPtr = nullptr;
	sta::Port* portPtr = nullptr;
	for(auto& portData : inBlockData.mPortDataVec) {
		if(portData.mBusFlag)
			portPtr = inNetworkPtr->makeBusPort(
					cellPtr, portData.mName.c_str(),
					portData.mRangeFrom, portData.mRangeTo);
		else
			portPtr = inNetworkPtr->makePort(
					cellPtr, portData.mName.c_str());

		portDirPtr = getPortDirection(portData);
		inNetworkPtr->setDirection(portPtr, portDirPtr);
	}

	return cellPtr;
}

/**
 * Returns direction of port depending on input and output flags.
 * Both are set = bidirect.
 * Input is set = input.
 * Output is set = output.
 * Both aren't set = unknown.
 * @param inPortData port data
 * @return port direction
 */
sta::PortDirection* OpenStaServerHandler::getPortDirection(
							const PortData& inPortData) {
	if(inPortData.mInput && inPortData.mOutput)
		return sta::PortDirection::bidirect();

	if(inPortData.mInput)
		return sta::PortDirection::input();

	if(inPortData.mOutput)
		return sta::PortDirection::output();

	return sta::PortDirection::unknown();
}

/**
 * Creates internal body of an instance.
 * Returns false if either input pointer except parent is null.
 * Parent instance can be null because of top-instance.
 * Appends exec messages if something goes wrong.
 * @param inNetworkPtr target network
 * @param inLibraryPtr target library
 * @param inParentPtr parent context instance
 * @param inParentData data of parent instance
 * @param inCurrInstPtr current instance
 * @param inCurrData data of current instance
 * @param inCellPtr master cell of current instance
 * @param inBlockDataVec data of all blocks
 * @param inExternalPinConnsVec external connectivity of instance
 * @return success flag
 */
bool OpenStaServerHandler::createBlockBody(
		sta::NetworkReader* inNetworkPtr,
		sta::Library* inLibraryPtr,
		sta::Instance* inParentPtr,
		const BlockData& inParentData,
		sta::Instance* inCurrInstPtr,
		const BlockData& inCurrData,
		sta::Cell* inCellPtr,
		const std::vector<BlockData>& inBlockDataVec,
		const std::vector<PortData>& inExternalPinConnsVec) {
	if(!inNetworkPtr || !inLibraryPtr || !inCurrInstPtr || !inCellPtr)
		return false;

	//first creating all internal nets, also marking vdd/gnd
	if(!createBlockBodyNets(
			inNetworkPtr, inLibraryPtr,
			inCurrInstPtr, inCurrData)) {
		addExecMessage("failed to create nets body of block: " + inCurrData.mName);
		return false;
	}

	//then creating internal pins of the ports (only single or bits!)
	//making them terminals of connected nets
	if(!createInstPinConns(
			inNetworkPtr, inLibraryPtr,
			inParentPtr, inParentData,
			inCurrInstPtr, inCurrData,
			inCellPtr, inExternalPinConnsVec)) {
		addExecMessage("failed to create instance body of block: " + inCurrData.mName);
		return false;
	}

	//if block is a leaf, then there's nothing to do with internal instances
	if(inCurrData.mLeafFlag)
		return true;


	//=========================================================================
	//creation of internal instances!
	bool instOk = true;

	//processing data for each instance
	for(auto& instData : inCurrData.mInstDataVec) {
		//cannot create instances of unknown block
		if(instData.mMasterBlockIdx >= inBlockDataVec.size()) {
			addExecMessage("instance without correct master block: " + instData.mName);
			instOk = false;
			continue;
		}

		//searching for master block in the network
		sta::Cell* cellPtr = findCreateBlockDeclaration(
					inNetworkPtr, inLibraryPtr,
					inBlockDataVec[instData.mMasterBlockIdx]);
		if(!cellPtr) {
			addExecMessage("instance without correct master block: " + instData.mName);
			instOk = false;
			continue;
		}


		//creating the instance
		sta::Instance* instPtr = inNetworkPtr->makeInstance(
				cellPtr, instData.mName.c_str(), inCurrInstPtr);
		//you shall not pass! no more than one top per netlist...
		if(!instPtr)
			continue;

		//and filling it's internals(nets, pin&connections, sub-insts)
		if(!createBlockBody(
				inNetworkPtr, inLibraryPtr,
				inCurrInstPtr, inCurrData,
				instPtr, inBlockDataVec[instData.mMasterBlockIdx],
				cellPtr, inBlockDataVec,
				instData.mPortDataVec)) {
			addExecMessage("failed to create subinstance body: " + instData.mName);
			instOk = false;
		}

	}

	return instOk;
}

/**
 * Locates/creates nets in the block's instance.
 * Also sets constant nets if they match names in block data.
 * Returns false if fails to create net of one of input pointers is null.
 * @param inNetworkPtr target network
 * @param inLibraryPtr target library
 * @param inParentInstPtr block's instance
 * @param inBlockData data of the block
 * @return success flag
 */
bool OpenStaServerHandler::createBlockBodyNets(
				sta::NetworkReader* inNetworkPtr,
				sta::Library* inLibraryPtr,
				sta::Instance* inParentInstPtr,
				const BlockData& inBlockData) {
	if(!inNetworkPtr || !inLibraryPtr || !inParentInstPtr)
		return false;

	bool allOk = true;
	sta::Net* netPtr = nullptr;

	//create all nets at once
	for(auto netName : inBlockData.mNetNamesVec) {
		netPtr = getCreateNet(
				inNetworkPtr, inParentInstPtr, netName);
		if(!netPtr) {
			setExecMessage("failed to create net " + netName);
			allOk = false;
			continue;
		}

		//registering constants
	    if (netName == inBlockData.mGndNetName)
	    	inNetworkPtr->addConstantNet(netPtr, sta::LogicValue::zero);

	    if (netName == inBlockData.mVddNetName)
	    	inNetworkPtr->addConstantNet(netPtr, sta::LogicValue::one);
	}

	return allOk;
}

/**
 * Creates all pins for the instance and connects to their nets.
 * Processes all pin bits and pins individually.
 * Uses \link createInstPinConnection to process each individual pin/bit.
 * Doesn't set up error messages, has only return flag.
 * Returns false if any input pointer except parent is null.
 * Returns false if fails to process one of pins.
 * @param inNetworkPtr target network
 * @param inLibraryPtr target library
 * @param inParentPtr parent context (null for top-instance)
 * @param inParentData data of parent instance (dummy for top-instance)
 * @param inCurrInstPtr current instance
 * @param inCurrData data of current instance (for internal connections)
 * @param inCellPtr master cell of current instance
 * @param inExternalPinConnsVec external connectivity of instance
 * @return success flag
 */
bool OpenStaServerHandler::createInstPinConns(
				sta::NetworkReader* inNetworkPtr,
				sta::Library* inLibraryPtr,
				sta::Instance* inParentPtr,
				const BlockData& inParentData,
				sta::Instance* inCurrInstPtr,
				const BlockData& inCurrData,
				sta::Cell* inCellPtr,
				const std::vector<PortData>& inExternalPinConnsVec) {
	if(!inNetworkPtr || !inLibraryPtr || !inCurrInstPtr || !inCellPtr)
		return false;

	bool allOk = true;

	std::string portName;
	size_t netIdx = 0;

	//processing all pins of the current instance
	for(size_t pinIdx = 0; pinIdx < inCurrData.mPortDataVec.size(); pinIdx++) {
		const PortData& currPortData = inCurrData.mPortDataVec[pinIdx];
		netIdx = 0;

		//for bus ports connecting all of it's sub-pins
		if(currPortData.mBusFlag && currPortData.mRangeFrom <= currPortData.mRangeTo) {
			//need to process all bits
			for(uint32_t bitIdx = currPortData.mRangeFrom; bitIdx <= currPortData.mRangeTo; bitIdx++) {
				//create a port name
				portName = makeBitName(inLibraryPtr, currPortData.mName, bitIdx);

				//and find-connect pin by bit-name of the port
				if(!createInstPinConnection(
						inNetworkPtr, inParentPtr,
						inCellPtr, inCurrData,
						inCurrInstPtr, portName,
						inParentData.mNetNamesVec,
						inExternalPinConnsVec,
						inCurrData.mNetNamesVec,
						inCurrData.mPortDataVec,
						pinIdx, netIdx)) {
					//yeah, conditional breakpoints do exist...
					allOk = false;
				}

				netIdx++;
			}
		} else {
			//for single pin process it by it's name
			if(!createInstPinConnection(
					inNetworkPtr, inParentPtr,
					inCellPtr, inCurrData,
					inCurrInstPtr, currPortData.mName,
					inParentData.mNetNamesVec,
					inExternalPinConnsVec,
					inCurrData.mNetNamesVec,
					inCurrData.mPortDataVec,
					pinIdx, netIdx)) {
				allOk = false;
			}
		}
	}

	return allOk;
}

/**
 * Makes single pin of instance, finds and connects it's net.
 * Connects pin on both outside and inside to transmit STA traversals.
 * Pins stays unconnected it there's no connected net.
 * Returns false if any input pointer except parent instance is null.
 * Returns false and adds report message if fails to find port in master cell.
 * @param inNetworkPtr target network
 * @param inParentInstPtr parent context (null for top-instance)
 * @param inCurrMasterCellPtr master cell of current instance
 * @param inCurrMasterBlockData data of master cell
 * @param inCurrInstPtr current instance
 * @param inPortName name of pin to create
 * @param inExternalNetNamesVec externally connected nets (previous level)
 * @param inExternalPinsDataVec data of external pins
 * @param inInternalNetNamesVec internally connected nets (current level)
 * @param inPinIdx index to select target pin data
 * @param inNetIdx index of netId in pin data
 * @return success flag
 */
bool OpenStaServerHandler::createInstPinConnection(
		sta::NetworkReader* inNetworkPtr,
		sta::Instance* inParentInstPtr,
		sta::Cell* inCurrMasterCellPtr,
		const BlockData& inCurrMasterBlockData,
		sta::Instance* inCurrInstPtr,
		const std::string& inPortName,
		const std::vector<std::string>& inExternalNetNamesVec,
		const std::vector<PortData>& inExternalPinsDataVec,
		const std::vector<std::string>& inInternalNetNamesVec,
		const std::vector<PortData>& inInternalPinsDataVec,
		uint32_t inPinIdx,
		uint32_t inNetIdx) {
	//parent inst isn't mandatory because there's a top-instance
	if(!inNetworkPtr || !inCurrMasterCellPtr || !inCurrInstPtr)
		return false;

	//find a port
	sta::Port* portPtr = inNetworkPtr->findPort(inCurrMasterCellPtr, inPortName.c_str());
	if(!portPtr) {
		setExecMessage("cannot create pin, no port with name " + inPortName);
		return false;
	}

	//find an externally-connected net
	sta::Net* externalNetPtr = findNetByIdx(
			inNetworkPtr, inParentInstPtr,
			inExternalNetNamesVec,
			inExternalPinsDataVec,
			inPinIdx, inNetIdx);

	//and the internal one
	sta::Net* internalNetPtr = findNetByIdx(
			inNetworkPtr, inCurrInstPtr,
			inInternalNetNamesVec,
			inInternalPinsDataVec,
			inPinIdx, inNetIdx);

	//do the connection depending on leaf-config
	bool cellIsLiberty = inNetworkPtr->libertyCell(inCurrMasterCellPtr) != nullptr;
	if(cellIsLiberty) {
		//for liberty-cells have to create pin
		inNetworkPtr->makePin(inCurrInstPtr, portPtr, externalNetPtr);
	} else {
		//if block comes from netlist but isn't hierarhical,
		//then there's no need to pin-connect, because timing isn't important here
		if(inCurrMasterBlockData.mLeafFlag) {
			inNetworkPtr->connect(inCurrInstPtr, portPtr, externalNetPtr);
		} else {
			//for hierarchical netlist blocks have to pin-connect
			sta::Pin *pin = inNetworkPtr->makePin(inCurrInstPtr, portPtr, externalNetPtr);

			//and transfer timing-search inside the instance through terminal(?)
			inNetworkPtr->makeTerm(pin, internalNetPtr);
		}
	}

	return true;
}

/**
 * Picks net name and searches it in context of parent instance.
 * First picks pin data by index, gets netId in pin data by net index.
 * Then uses this net ID to get net name from names vector.
 * Needed this to compress a little size of messages.
 * Returns nullptr if one of indexes is out of bounds.
 * Returns nullptr if either network if instance is null.
 * Returns nullptr if fails to find name by it's index if instance has no net.
 * @param inNetworkPtr target network
 * @param inParentInstPtr context instance
 * @param inNetNamesVec net names vector
 * @param inPinsDataVec pin data
 * @param inPinIdx index of target pin
 * @param inNetIdx index of net in pin data
 */
sta::Net* OpenStaServerHandler::findNetByIdx(
		sta::NetworkReader* inNetworkPtr,
		sta::Instance* inParentInstPtr,
		const std::vector<std::string>& inNetNamesVec,
		const std::vector<PortData>& inPinsDataVec,
		uint32_t inPinIdx,
		uint32_t inNetIdx) {
	//can't take nets out from nowhere
	if(!inNetworkPtr || !inParentInstPtr)
		return nullptr;

	//all data must be within range,
	//otherwise just return nullptr for the top-level
	if(inPinIdx >= inPinsDataVec.size())
		return nullptr;

	if(inNetIdx >= inPinsDataVec[inPinIdx].mConnNetIdxsVec.size())
		return nullptr;

	uint32_t netIdx = inPinsDataVec[inPinIdx].mConnNetIdxsVec[inNetIdx];
	if(netIdx >= inNetNamesVec.size())
		return nullptr;

	return inNetworkPtr->findNet(inParentInstPtr, inNetNamesVec[netIdx].c_str());
}

/**
 * Searches net in instance context, creates one if is doesn't exist.
 * Returns nullptr if netowrk of instance pointer is null.
 * @param inNetworkPtr target network
 * @param inParentInstPtr instance context
 * @param inName net name
 * @return net pointer
 */
sta::Net* OpenStaServerHandler::getCreateNet(
	    sta::NetworkReader* inNetworkPtr,
		sta::Instance* inParentInstPtr,
		const std::string& inName) {
	if(!inNetworkPtr || !inParentInstPtr)
		return nullptr;

	//getting net pointer or it's final assignee
	sta::Net* netPtr = inNetworkPtr->findNet(inParentInstPtr, inName.c_str());
	while (netPtr && inNetworkPtr->mergedInto(netPtr))
		netPtr = inNetworkPtr->mergedInto(netPtr);

	if(!netPtr)
		netPtr = inNetworkPtr->makeNet(inName.c_str(), inParentInstPtr);

	return netPtr;
}

/**
 * Composes bit name from base name and index.
 * Casts library and uses left and right brackets to do so.
 * Returns empty string if library is null.
 * @param inLibraryPtr target library
 * @param inBaseName base name
 * @param inIdx bit index
 * @return bit name or empty string
 */
std::string OpenStaServerHandler::makeBitName(
		sta::Library* inLibraryPtr,
		std::string inBaseName,
		uint32_t inIdx) {
	sta::ConcreteLibrary* cnrLibPtr =
			reinterpret_cast<sta::ConcreteLibrary*>(inLibraryPtr);
	if(!cnrLibPtr)
		return "";

	std::stringstream ss;
	ss << inBaseName << cnrLibPtr->busBrktLeft()
			<< inIdx << cnrLibPtr->busBrktRight();
	return ss.str();
}


/**
 * Searches the net in instance context.
 * Assuming name of the top-instance in OpenSTA isn't included in the context.
 * Returns nullptr if fails to find context instance or net.
 * Returns nullptr if there's no top-instance or network is null.
 * @param inNetworkPtr target network
 * @param mInstNamesContextVec top-bottom inst names
 * @param inNetName name of target net
 * @return net pointer or nullptr
 */
sta::Net* OpenStaServerHandler::findNetInContext(
		sta::Network* inNetworkPtr,
		const std::vector<std::string>& mInstNamesContextVec,
		const std::string& inNetName) {
	if(!inNetworkPtr)
		return nullptr;

	sta::Instance* currInstPtr = inNetworkPtr->topInstance();
	if(!currInstPtr)
		return nullptr;

	//descending into the inst hierarchy
	for(const std::string& instName : mInstNamesContextVec) {
		currInstPtr = inNetworkPtr->findChild(
				currInstPtr, instName.c_str());
		if(!currInstPtr)
			break;
	}

	//nothing to do without final context
	if(!currInstPtr)
		return nullptr;

	return inNetworkPtr->findNet(currInstPtr, inNetName.c_str());
}

/**
 * Connects/disconnects pin and net.
 * If pin is already connected and has to connect, then returns true.
 * If pin is already connected and has to disconnect, then disconnects first.
 * Name of target instance may be empty if targets pin in context's inst.
 * Returns false if sta or design aren't ready.
 * Returns false if fails to find pin or net.
 * @param inInstContextVec context instance path
 * @param inNetName net name
 * @param inInstName instance name
 * @param inPinName target pin name
 * @param inConnect flag to connect/disconnect
 */
bool OpenStaServerHandler::connDisPinNet(
						const std::vector<std::string>& inInstContextVec,
						const std::string& inNetName,
						const std::string& inInstName,
						const std::string& inPinName,
						bool inConnect) {
	if(!sta::Sta::sta())
		return false;

	sta::Network* networkPtr = sta::Sta::sta()->cmdNetwork();
	if (!networkPtr ||
			!networkPtr->isLinked() ||
			!networkPtr->topInstance())
		return false;

	//searching context instance
	//doing it separately not to copy and modify context path
	sta::Instance* contextInstPtr = findContextInstance(
			networkPtr, networkPtr->topInstance(), inInstContextVec);
	if(!contextInstPtr)
		return false;

	//searching the target pin
	sta::Instance* instPtr = contextInstPtr;
	if(!inInstName.empty())
		instPtr = networkPtr->findChild(
			contextInstPtr, inInstName.c_str());
	if(!instPtr)
		return false;

	sta::Pin* pinPtr = networkPtr->findPin(
			instPtr, inPinName.c_str());
	if(!pinPtr)
		return false;

	//can disconnect without the net
	if(!inConnect) {
		sta::Sta::sta()->disconnectPin(pinPtr);
		return true;
	}


	//searching the target net to connect
	sta::Net* netPtr = networkPtr->findNet(
			contextInstPtr, inNetName.c_str());
	if(!netPtr)
		return false;

	//checking where pin is connected right now
	//disconnecting if necessary
	sta::Net* connNetPtr = networkPtr->net(pinPtr);
	if(connNetPtr) {
		if(connNetPtr == netPtr)
			return true;

		sta::Sta::sta()->disconnectPin(pinPtr);
	}

	sta::Port* portPtr = networkPtr->port(pinPtr);
	if(!portPtr)
		return false;

	sta::Sta::sta()->connectPin(instPtr, portPtr, netPtr);
	return true;
}


/**
 * Writes out pin name and top to bottom instance hierarchy
 * excluding top instance.
 * Does nothing and returns false if either input pointer is null.
 * @param inPinPtr target pin
 * @param outName pin name to fill
 * @param outHierInstsVec instance context to fill
 * @return operations success
 */
bool OpenStaServerHandler::fillPinPaths(
		sta::Network* inNetworkPtr,
		sta::Pin* inPinPtr,
		std::string& outName,
		std::vector<std::string>& outHierInstsVec) {
	if(!inPinPtr)
		return false;

	outName = inNetworkPtr->portName(inPinPtr);

	sta::Instance* instPtr = inNetworkPtr->instance(inPinPtr);
	while(instPtr && instPtr != inNetworkPtr->topInstance()) {
		outHierInstsVec.push_back(
				inNetworkPtr->name(instPtr));
		instPtr = inNetworkPtr->parent(instPtr);
	}

	//flipping names order
	std::reverse(outHierInstsVec.begin(), outHierInstsVec.end());

	return true;
}

/**
 * Dives in instance hierarchy to find context instance.
 * Returns top-instance if network is null or path is empty.
 * Returns nullptr if at some point fails to find instance.
 * @param inNetworkPtr network
 * @param inTopInstPtr top-level instance
 * @param inObjPathVec top-down inst names
 * @return context instance
 */
sta::Instance* OpenStaServerHandler::findContextInstance(
			sta::Network* inNetworkPtr,
			sta::Instance* inTopInstPtr,
			const std::vector<std::string>& inObjPathVec) {
	if(!inNetworkPtr || inObjPathVec.empty())
		return inTopInstPtr;

	sta::Instance* contextInstPtr = inTopInstPtr;
	for(const std::string& instName : inObjPathVec) {
		if(!contextInstPtr)
			return nullptr;

		contextInstPtr = inNetworkPtr->findChild(contextInstPtr, instName.c_str());
	}

	return contextInstPtr;
}

/**
 * Returns setup/hold pointer depending on flags.
 * Returns both if they are true/true or false/false.
 * @param inSetup setup flag
 * @param inHold hold flag
 * @return setup/fall/hold pointer
 */
sta::SetupHoldAll* OpenStaServerHandler::getSetupHoldAll(
						bool inSetup,
						bool inHold) {
	//setup=max, hold=min
	return getMinMaxAll(inHold, inSetup);
}

/**
 * Returns rise/fall pointer depending on flags.
 * Returns both if they are true/true or false/false.
 * @param inRise rise flag
 * @param inFall fall flag
 * @return rise/fall/both pointer
 */
sta::RiseFallBoth* OpenStaServerHandler::getRiseFallBoth(
						bool inRise,
						bool inFall) {
	sta::RiseFallBoth* rfPtr = sta::RiseFallBoth::riseFall();
	if(inRise && !inFall)
		rfPtr = sta::RiseFallBoth::rise();
	if(!inRise && inFall)
		rfPtr = sta::RiseFallBoth::fall();

	return rfPtr;
}

/**
 * Returns min/max pointer depending on flags.
 * Returns both if they are true/true or false/false.
 * @param inMin min flag
 * @param inMax max flag
 * @return min/max/both pointer
 */
sta::MinMaxAll* OpenStaServerHandler::getMinMaxAll(
						bool inMin,
						bool inMax) {
	sta::MinMaxAll* minMaxPtr = sta::MinMaxAll::all();
	if(inMin && !inMax)
		minMaxPtr = sta::MinMaxAll::min();
	if(!inMin && inMax)
		minMaxPtr = sta::MinMaxAll::max();

	return minMaxPtr;
}

/**
 * Returns early/late pointer depending on flags.
 * Returns both if they are true/true or false/false.
 * @param inEarly early flag
 * @param inLate late flag
 * @return early/late/both pointer
 */
sta::MinMaxAll* OpenStaServerHandler::getEarlyLateAll(
						bool inEarly,
						bool inLate) {
	sta::MinMaxAll* earlyLatePtr = sta::MinMaxAll::all();
	if(inEarly && !inLate)
		earlyLatePtr = sta::MinMaxAll::early();
	if(!inEarly && inLate)
		earlyLatePtr = sta::MinMaxAll::late();

	return earlyLatePtr;
}


/**
 * Locates from pins, clocks and instances of paths specification.
 * Returns exception if at least one entity exists.
 * Otherwise returns nullptr.
 * @param inCommand command to process
 * @return exception or nullptr
 */
sta::ExceptionFrom* OpenStaServerHandler::createPathExceptionFrom(
						sta::Sdc* inSdcPtr,
						sta::Network* inNetworkPtr,
						sta::Instance* inTopInstPtr,
						const TimingPathMessageBase& inCommand) {
	sta::PinSet* pinsSetPtr = new sta::PinSet();
	findAddObjectsByAddr<sta::Pin, TargetSearchObjType::SearchObjTypePin>(
			inNetworkPtr, inTopInstPtr,
			inCommand.mFromPinPathsVec, pinsSetPtr);

	sta::InstanceSet* instsSetPtr = new sta::InstanceSet();
	findAddObjectsByAddr<sta::Instance, TargetSearchObjType::SearchObjTypeInst>(
			inNetworkPtr, inTopInstPtr,
			inCommand.mFromPinPathsVec, instsSetPtr);

	sta::ClockSet* clocksSetPtr = new sta::ClockSet();
	fillClocksInSet(inSdcPtr, inCommand.mFromClocksVec, clocksSetPtr);

	sta::RiseFallBoth* rfPtr = getRiseFallBoth(
			inCommand.mFromRise, inCommand.mFromFall);

	//if all points are empty, then returning null
	if(pinsSetPtr->empty() &&
			instsSetPtr->empty() &&
			clocksSetPtr->empty()) {
		delete pinsSetPtr;
		delete instsSetPtr;
		delete clocksSetPtr;
		return nullptr;
	}

	return sta::Sta::sta()->makeExceptionFrom(
			pinsSetPtr, clocksSetPtr,
			instsSetPtr, rfPtr);
}

/**
 * Locates through pins, clocks and instances of paths specification.
 * Returns exception if at least one entity exists.
 * Otherwise returns nullptr.
 * Creates single thru-entry from all the instances, pins and nets.
 * @param inCommand command to process
 * @return exception or nullptr
 */
sta::ExceptionThruSeq* OpenStaServerHandler::createPathExceptionThrus(
						sta::Network* inNetworkPtr,
						sta::Instance* inTopInstPtr,
						const TimingPathMessageBase& inCommand) {
	sta::PinSet* pinsSetPtr = new sta::PinSet();
	findAddObjectsByAddr<sta::Pin, TargetSearchObjType::SearchObjTypePin>(
			inNetworkPtr, inTopInstPtr,
			inCommand.mThroughPinPathsVec, pinsSetPtr);

	sta::NetSet* netsSetPtr = new sta::NetSet();
	findAddObjectsByAddr<sta::Net, TargetSearchObjType::SearchObjTypeNet>(
			inNetworkPtr, inTopInstPtr,
			inCommand.mThroughNetPathsVec, netsSetPtr);

	sta::InstanceSet* instsSetPtr = new sta::InstanceSet();
	findAddObjectsByAddr<sta::Instance, TargetSearchObjType::SearchObjTypeInst>(
			inNetworkPtr, inTopInstPtr,
			inCommand.mThroughInstPathsVec, instsSetPtr);

	sta::RiseFallBoth* rfPtr = getRiseFallBoth(
			inCommand.mThroughRise, inCommand.mThroughFall);

	//if all points are empty, then returning null
	if(pinsSetPtr->empty() &&
			instsSetPtr->empty() &&
			netsSetPtr->empty()) {
		delete pinsSetPtr;
		delete instsSetPtr;
		delete netsSetPtr;
		return nullptr;
	}

	sta::ExceptionThruSeq* trusVec = new sta::ExceptionThruSeq();
	trusVec->push_back(
			sta::Sta::sta()->makeExceptionThru(
					pinsSetPtr, netsSetPtr, instsSetPtr, rfPtr));

	return trusVec;
}

/**
 * Locates to pins, clocks and instances of paths specification.
 * Returns exception if at least one entity exists.
 * Otherwise returns nullptr.
 * Sets "end" rise/fall to "to" rise/fall.
 * @param inCommand command to process
 * @return exception or nullptr
 */
sta::ExceptionTo* OpenStaServerHandler::createPathExceptionTo(
						sta::Sdc* inSdcPtr,
						sta::Network* inNetworkPtr,
						sta::Instance* inTopInstPtr,
						const TimingPathMessageBase& inCommand) {
	sta::PinSet* pinsSetPtr = new sta::PinSet();
	findAddObjectsByAddr<sta::Pin, TargetSearchObjType::SearchObjTypePin>(
			inNetworkPtr, inTopInstPtr,
			inCommand.mToPinPathsVec, pinsSetPtr);

	sta::InstanceSet* instsSetPtr = new sta::InstanceSet();
	findAddObjectsByAddr<sta::Instance, TargetSearchObjType::SearchObjTypeInst>(
			inNetworkPtr, inTopInstPtr,
			inCommand.mToPinPathsVec, instsSetPtr);

	sta::ClockSet* clocksSetPtr = new sta::ClockSet();
	fillClocksInSet(inSdcPtr, inCommand.mToClocksVec, clocksSetPtr);

	sta::RiseFallBoth* rfPtr = getRiseFallBoth(
			inCommand.mToRise, inCommand.mToFall);

	//if all points are empty, then returning null
	if(pinsSetPtr->empty() &&
			instsSetPtr->empty() &&
			clocksSetPtr->empty()) {
		delete pinsSetPtr;
		delete instsSetPtr;
		delete clocksSetPtr;
		return nullptr;
	}

	return sta::Sta::sta()->makeExceptionTo(
			pinsSetPtr, clocksSetPtr, instsSetPtr, rfPtr, rfPtr);
}

/**
 * Collects clocks by their names in output set.
 * Returns false if sdc or output set is null.
 * @param inSdcPtr sdc to search clocks
 * @param inNamesVec clock names
 * @param outClocksSetPtr set where to write out clocks
 * @return success status
 */
bool OpenStaServerHandler::fillClocksInSet(
						sta::Sdc* inSdcPtr,
						const std::vector<std::string>& inNamesVec,
						sta::ClockSet* outClocksSetPtr) {
	if(!inSdcPtr || !outClocksSetPtr)
		return false;

	sta::Clock* clockPtr = nullptr;

	//processing all clock names, adding existent ones
	for(const std::string& clockName : inNamesVec) {
		clockPtr = inSdcPtr->findClock(clockName.c_str());
		if(!clockPtr)
			continue;

		outClocksSetPtr->insert(clockPtr);
	}

	return true;
}



}
