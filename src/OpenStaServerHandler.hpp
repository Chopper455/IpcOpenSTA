#ifndef SRC_OPENSTASERVERHANDLER_HPP_
#define SRC_OPENSTASERVERHANDLER_HPP_

#include "common/IMessageExecutor.hpp"

#include "Sta.hh"
#include "Network.hh"
#include "ConcreteLibrary.hh"
#include "PortDirection.hh"

#include <tcl.h>


#include <sstream>


namespace stamask {

/**
 * Executes OpenSTA engine's procedures according to input commands.
 */
class OpenStaServerHandler : public stamask::IMessageExecutor {

	/** TCL interpreter for OpenSTA */
	Tcl_Interp * mTclInterp;

	/** OpenSTA god and everything */
	sta::Sta* mStaPtr;

	/** name of library where to compose netlist from blocks data */
	const std::string mcNetlistLibName = "verilog";

	/** return message to create when executing command */
	std::string mExecResultMessage;

public:


	OpenStaServerHandler();

	virtual ~OpenStaServerHandler();

	virtual std::string getExecMessage() const;

	virtual bool execute(
			const CommandSetHierarhySeparator& inCommand);
	virtual bool execute(
			const CommandExit& inCommand);
	virtual bool execute(
			const CommandPing& inCommand);
	virtual bool execute(
			const CommandReadLibertyFile& inCommand);
	virtual bool execute(
			const CommandReadLibertyStream& inCommand);
	virtual bool execute(
			const CommandClearLibs& inCommand);
	virtual bool execute(
			const CommandReadVerilogFile& inCommand);
	virtual bool execute(
			const CommandReadVerilogStream& inCommand);
	virtual bool execute(
			const CommandLinkTop& inCommand);
	virtual bool execute(
			const CommandClearNetlistBlocks& inCommand);
	virtual bool execute(
			const CommandCreateNetlist& inCommand);
	virtual bool execute(
			const CommandGetGraphData& inCommand,
			std::vector<VertexIdData>& outVertexIdToDataVec,
			std::vector<EdgeIdData>& outEdgeIdToDataVec);
	virtual bool execute(
			const CommandGetGraphSlacksData& inCommand,
			std::vector<NodeTimingData>& outNodeTimingsVec);
	virtual bool execute(
			const CommandSetArcsDelays& inCommand);
	virtual bool execute(
			const CommandConnectContextPinNet& inCommand);
	virtual bool execute(
			const CommandDisconnectContextPinNet& inCommand);
	virtual bool execute(
			const CommandReadSpefFile& inCommand);
	virtual bool execute(
			const CommandReadSpefStream& inCommand);
	virtual bool execute(
			const CommandSetGroupNetCap& inCommand);
	virtual bool execute(
			const CommandReadSdfFile& inCommand);
	virtual bool execute(
			const CommandReadSdfStream& inCommand);
	virtual bool execute(
			const CommandWriteSdfFile& inCommand);
	virtual bool execute(
			const CommandReportTiming& inCommand,
			std::string& outReportStr);


	virtual bool execute(
			const CommandCreateClock& inCommand);
	virtual bool execute(
			const CommandCreateGenClock& inCommand);
	virtual bool execute(
			const CommandSetClockGroups& inCommand);
	virtual bool execute(
			const CommandSetClockLatency& inCommand);
	virtual bool execute(
			const CommandSetInterClockUncertainty& inCommand);
	virtual bool execute(
			const CommandSetSingleClockUncertainty& inCommand);
	virtual bool execute(
			const CommandSetSinglePinUncertainty& inCommand);
	virtual bool execute(
			const CommandSetPortDelay& inCommand);
	virtual bool execute(
			const CommandSetInPortTransition& inCommand);
	virtual bool execute(
			const CommandSetFalsePath& inCommand);
	virtual bool execute(
			const CommandSetMinMaxDelay& inCommand);
	virtual bool execute(
			const CommandSetMulticyclePath& inCommand);
	virtual bool execute(
			const CommandDisableSinglePinTiming& inCommand);
	virtual bool execute(
			const CommandDisableInstTiming& inCommand);
	virtual bool execute(
			const CommandSetGlobalTimingDerate& inCommand);


private:

	void setExecMessage(const std::string& inMessage);

	void addExecMessage(const std::string& inMessage);

	bool checkStaReady();

	bool checkDesignLoaded();




	uint32_t findTopBlockIdx(
			const std::vector<BlockData>& inBlocksDataVec);

	sta::Library* getCreateNetlistLib(
			sta::NetworkReader* inNetworkPtr);

	sta::Cell* findCreateBlockDeclaration(
			sta::NetworkReader* inNetworkPtr,
			sta::Library* inLibraryPtr,
			const BlockData& inBlockData);

	sta::Cell* createBlockDeclaration(
			sta::NetworkReader* inNetworkPtr,
			sta::Library* inLibraryPtr,
			const BlockData& inBlockData);

	sta::PortDirection* getPortDirection(
			const PortData& inPortData);

	bool createBlockBody(
			sta::NetworkReader* inNetworkPtr,
			sta::Library* inLibraryPtr,
			sta::Instance* inParentPtr,
			const BlockData& inParentData,
			sta::Instance* inCurrInstPtr,
			const BlockData& inCurrData,
			sta::Cell* inCellPtr,
			const std::vector<BlockData>& inBlockDataVec,
			const std::vector<PortData>& inExternalPinConnsVec);

	bool createBlockBodyNets(
					sta::NetworkReader* inNetworkPtr,
					sta::Library* inLibraryPtr,
					sta::Instance* inParentInstPtr,
					const BlockData& inBlockData);

	bool createInstPinConns(
					sta::NetworkReader* inNetworkPtr,
					sta::Library* inLibraryPtr,
					sta::Instance* inParentPtr,
					const BlockData& inParentData,
					sta::Instance* inCurrInstPtr,
					const BlockData& inCurrData,
					sta::Cell* inCellPtr,
					const std::vector<PortData>& inExternalPinConnsVec);

	bool createInstPinConnection(
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
			uint32_t inNetIdx);

	sta::Net* findNetByIdx(
			sta::NetworkReader* inNetworkPtr,
			sta::Instance* inParentInstPtr,
			const std::vector<std::string>& inNetNamesVec,
			const std::vector<PortData>& inPinsDataVec,
			uint32_t inPinIdx,
			uint32_t inNetIdx);



	sta::Net* getCreateNet(
		    sta::NetworkReader* inNetworkPtr,
			sta::Instance* inParentInstPtr,
			const std::string& inName);


	std::string makeBitName(
			sta::Library* inLibraryPtr,
			std::string inBaseName,
			uint32_t inIdx);

	sta::Net* findNetInContext(
			sta::Network* inNetworkPtr,
			const std::vector<std::string>& mInstNamesContextVec,
			const std::string& inNetName);

	bool connDisPinNet(
			const std::vector<std::string>& inInstContextVec,
			const std::string& inNetName,
			const std::string& inInstName,
			const std::string& inPinName,
			bool inConnect);


private:

	bool collectVertexesData(
			sta::Network* inNetworkPtr,
			sta::Graph* inGraphPtr,
			std::vector<uint32_t>& outVertexIdToIdxVec,
			std::vector<VertexIdData>& inVertexIdToDataVec);

	bool collectEdgesData(
			sta::Graph* inGraphPtr,
			const std::vector<uint32_t>& inVertexIdToIdxVec,
			std::vector<EdgeIdData>& outEdgeIdToDataVec);


	bool fillPinPaths(
			sta::Network* inNetworkPtr,
			sta::Pin* inPinPtr,
			std::string& outName,
			std::vector<std::string>& outHierInstsVec);

	bool fillGraphNodeAatRat(
			sta::Sta* inStaPtr,
			sta::Vertex* inVertexPtr,
			NodeTimingData& outNodeData);

	bool fillSetBackpathNodesRat(
			sta::Sta* inStaPtr,
			sta::Graph* inGraphPtr,
			sta::Vertex* inVertexPtr,
			std::vector<NodeTimingData>& outDataVec,
			uint32_t inEndPointGroupId,
			bool inMinCondition);

	float getSlack(
			const NodeTimingData& inData,
			bool inMinCondition);

private:

	sta::Instance* findContextInstance(
			sta::Network* inNetworkPtr,
			sta::Instance* inTopInstPtr,
			const std::vector<std::string>& inObjPathVec);

	sta::SetupHoldAll* getSetupHoldAll(
			bool inSetup,
			bool inHold);

	sta::RiseFallBoth* getRiseFallBoth(
			bool inRise,
			bool inFall);

	sta::MinMaxAll* getMinMaxAll(
			bool inMin,
			bool inMax);

	sta::MinMaxAll* getEarlyLateAll(
			bool inEarly,
			bool inLate);

	bool findAddPins(
			sta::Network* inNetworkPtr,
			sta::Instance* inParentInstPtr,
			const std::vector<ObjectContextNameData>& mPinPathsVec,
			sta::PinSet* outPinSetPtr);

	enum class TargetSearchObjType {
		SearchObjTypePin,
		SearchObjTypeNet,
		SearchObjTypeInst
	};


	sta::ExceptionFrom* createPathExceptionFrom(
			sta::Sdc* inSdcPtr,
			sta::Network* inNetworkPtr,
			sta::Instance* inTopInstPtr,
			const TimingPathMessageBase& inCommand);

	sta::ExceptionThruSeq* createPathExceptionThrus(
			sta::Network* inNetworkPtr,
			sta::Instance* inTopInstPtr,
			const TimingPathMessageBase& inCommand);

	sta::ExceptionTo* createPathExceptionTo(
			sta::Sdc* inSdcPtr,
			sta::Network* inNetworkPtr,
			sta::Instance* inTopInstPtr,
			const TimingPathMessageBase& inCommand);


	bool fillClocksInSet(
			sta::Sdc* inSdcPtr,
			const std::vector<std::string>& inNamesVec,
			sta::ClockSet* outClocksSetPtr);




	template<typename _ObjType, TargetSearchObjType _objType>
	_ObjType* findObjInContext(
							sta::Network* inNetworkPtr,
							sta::Instance* inTopInstPtr,
							const std::vector<std::string>& inInstContextVec,
							const std::string& inObjName);


	template<typename _ObjType, OpenStaServerHandler::TargetSearchObjType _objType>
	bool findAddObjectsByAddr(
							sta::Network* inNetworkPtr,
							sta::Instance* inTopInstPtr,
							const std::vector<ObjectContextNameData>& mObjPathsVec,
							sta::Set<_ObjType*>* outObjSetPtr);

};


/**
 * First finds context instance, then finds object there.
 * Returns nullptr if fails to find instance context.
 * WARNING: object types in template must match because method uses casting.
 * @param inNetworkPtr network
 * @param inTopInstPtr top-instance
 * @param inInstContextVec path of the context instance
 * @param inObjName object name
 * @return object pointer or nullptr
 */
template<typename _ObjType, OpenStaServerHandler::TargetSearchObjType _objType>
_ObjType* OpenStaServerHandler::findObjInContext(
						sta::Network* inNetworkPtr,
						sta::Instance* inTopInstPtr,
						const std::vector<std::string>& inInstContextVec,
						const std::string& inObjName) {
	sta::Instance* instPtr = findContextInstance(
			inNetworkPtr, inTopInstPtr, inInstContextVec);
	if(!instPtr)
		return nullptr;


	//and another one shoots the leg
	switch(_objType) {
		case TargetSearchObjType::SearchObjTypePin:
			//if context instance is the top-level, then searching the same way as TCL does
			if(instPtr == inTopInstPtr)
				return (_ObjType*) inNetworkPtr->findPin(inObjName.c_str());

			return (_ObjType*) inNetworkPtr->findPin(instPtr, inObjName.c_str());
		case TargetSearchObjType::SearchObjTypeNet:
			return (_ObjType*) inNetworkPtr->findNet(instPtr, inObjName.c_str());
		case TargetSearchObjType::SearchObjTypeInst:
			return (_ObjType*) inNetworkPtr->findChild(instPtr, inObjName.c_str());
	}

	return nullptr;
}


/**
 * Find and collects objects by their paths.
 * Returns false if network, top-instance or set is null.
 * Returns false if fails to find one of objects.
 * @param inNetworkPtr network
 * @param inTopInstPtr top-instance
 * @param mObjPathsVec paths of objects
 * @param outObjSetPtr output container for objects
 * @return success flag
 */
template<typename _ObjType, OpenStaServerHandler::TargetSearchObjType _objType>
bool OpenStaServerHandler::findAddObjectsByAddr(
						sta::Network* inNetworkPtr,
						sta::Instance* inTopInstPtr,
						const std::vector<ObjectContextNameData>& mObjPathsVec,
						sta::Set<_ObjType*>* outObjSetPtr) {
	if(!inNetworkPtr || !inTopInstPtr || !outObjSetPtr)
		return false;

	bool allOk = true;
	_ObjType* objPtr = nullptr;

	//processing all pin paths
	for(const ObjectContextNameData& objPath : mObjPathsVec) {
		objPtr = findObjInContext<_ObjType, _objType>(
				inNetworkPtr, inTopInstPtr,
				objPath.mInstContextVec, objPath.mObjName);
		if(!objPtr) {
			allOk = false;
			continue;
		}

		outObjSetPtr->insert(objPtr);
	}

	return allOk;
}



}



#endif /* SRC_SERVER_DUMMYSTASERVERHANDLER_HPP_ */
