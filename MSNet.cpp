/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2017 German Aerospace Center (DLR) and others.
/****************************************************************************/
//
//   This program and the accompanying materials
//   are made available under the terms of the Eclipse Public License v2.0
//   which accompanies this distribution, and is available at
//   http://www.eclipse.org/legal/epl-v20.html
//
/****************************************************************************/
/// @file    MSNet.cpp
/// @author  Christian Roessel
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Clemens Honomichl
/// @author  Eric Nicolay
/// @author  Mario Krumnow
/// @author  Michael Behrisch
/// @author  Mario Krumnow
/// @author  Christoph Sommer
/// @date    Tue, 06 Mar 2001
/// @version $Id$
///
// The simulated network and simulation perfomer
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <string>
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <algorithm>
#include <cassert>
#include <vector>
#include <ctime>

#include "trigger/MSTrigger.h"
#include "trigger/MSCalibrator.h"
#include "traffic_lights/MSTLLogicControl.h"
#include "MSVehicleControl.h"
#include <utils/common/MsgHandler.h>
#include <utils/common/ToString.h>
#include <utils/common/SysUtils.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/WrappingCommand.h>
#include <utils/common/SystemFrame.h>
#include <utils/geom/GeoConvHelper.h>
#include <utils/iodevices/OutputDevice_File.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/options/OptionsCont.h>
#include <utils/options/OptionsIO.h>
#include <utils/shapes/ShapeContainer.h>
#include <utils/vehicle/DijkstraRouter.h>
#include <utils/vehicle/AStarRouter.h>
#include <utils/vehicle/PedestrianRouter.h>
#include <utils/xml/SUMORouteLoaderControl.h>
#include <utils/xml/XMLSubSys.h>
#include <mesosim/MELoop.h>
#include <microsim/output/MSDetectorControl.h>
#include <microsim/MSCModel_NonInteracting.h>
#include <microsim/MSVehicleTransfer.h>
#include <microsim/devices/MSDevice_Routing.h>
#include <microsim/devices/MSDevice_Vehroutes.h>
#include <microsim/devices/MSDevice_Tripinfo.h>
#include <microsim/devices/MSDevice_BTsender.h>
#include <microsim/devices/MSDevice_SSM.h>
#include <microsim/output/MSBatteryExport.h>
#include <microsim/output/MSEmissionExport.h>
#include <microsim/output/MSFCDExport.h>
#include <microsim/output/MSFullExport.h>
#include <microsim/output/MSQueueExport.h>
#include <microsim/output/MSVTKExport.h>
#include <microsim/output/MSXMLRawOut.h>
#include <microsim/output/MSAmitranTrajectories.h>
#include <microsim/pedestrians/MSPModel.h>
#include <microsim/pedestrians/MSPerson.h>
#include <microsim/traffic_lights/MSTrafficLightLogic.h>
#include <microsim/trigger/MSChargingStation.h>

#include "MSTransportableControl.h"
#include "MSEdgeControl.h"
#include "MSJunctionControl.h"
#include "MSInsertionControl.h"
#include "MSEventControl.h"
#include "MSEdge.h"
#include "MSJunction.h"
#include "MSJunctionLogic.h"
#include "MSLane.h"
#include "MSVehicleTransfer.h"
#include "MSRoute.h"
#include "MSGlobals.h"
#include "MSContainer.h"
#include "MSEdgeWeightsStorage.h"
#include "MSStateHandler.h"
#include "MSFrame.h"
#include "MSParkingArea.h"
#include "MSStoppingPlace.h"
#include "MSNet.h"


#include <3dParty/collisionDetected.h>
#include <libsumo//Vehicle.h>

#ifndef NO_TRACI
#include <traci-server/TraCIServer.h>
#include <libsumo/Simulation.h>
#endif


// ===========================================================================
// debug constants
// ===========================================================================
//#define DEBUG_SIMSTEP


// ===========================================================================
// static member definitions
// ===========================================================================
MSNet* MSNet::myInstance = 0;

const std::string MSNet::STAGE_EVENTS("events");
const std::string MSNet::STAGE_MOVEMENTS("move");
const std::string MSNet::STAGE_LANECHANGE("laneChange");
const std::string MSNet::STAGE_INSERTIONS("insertion");

// ===========================================================================
// static member method definitions
// ===========================================================================
double
MSNet::getEffort(const MSEdge* const e, const SUMOVehicle* const v, double t) {
	double value;
	const MSVehicle* const veh = dynamic_cast<const MSVehicle* const>(v);
	if (veh != 0 && veh->getWeightsStorage().retrieveExistingEffort(e, t, value)) {
		return value;
	}
	if (getInstance()->getWeightsStorage().retrieveExistingEffort(e, t, value)) {
		return value;
	}
	return 0;
}


double
MSNet::getTravelTime(const MSEdge* const e, const SUMOVehicle* const v, double t) {
	double value;
	const MSVehicle* const veh = dynamic_cast<const MSVehicle* const>(v);
	if (veh != 0 && veh->getWeightsStorage().retrieveExistingTravelTime(e, t, value)) {
		return value;
	}
	if (getInstance()->getWeightsStorage().retrieveExistingTravelTime(e, t, value)) {
		return value;
	}
	return e->getMinimumTravelTime(v);
}


// ---------------------------------------------------------------------------
// MSNet - methods
// ---------------------------------------------------------------------------
MSNet*
MSNet::getInstance(void) {
	if (myInstance != 0) {
		return myInstance;
	}
	throw ProcessError("A network was not yet constructed.");
}


MSNet::MSNet(MSVehicleControl* vc, MSEventControl* beginOfTimestepEvents,
	MSEventControl* endOfTimestepEvents, MSEventControl* insertionEvents,
	ShapeContainer* shapeCont) :
	myVehiclesMoved(0),
	myHavePermissions(false),
	myHasInternalLinks(false),
	myHasElevation(false),
	myRouterTT(0),
	myRouterEffort(0),
	myPedestrianRouter(0),
	myIntermodalRouter(0) {
	if (myInstance != 0) {
		throw ProcessError("A network was already constructed.");
	}
	OptionsCont& oc = OptionsCont::getOptions();
	myStep = string2time(oc.getString("begin"));
	myMaxTeleports = oc.getInt("max-num-teleports");
	myLogExecutionTime = !oc.getBool("no-duration-log");
	myLogStepNumber = !oc.getBool("no-step-log");
	myInserter = new MSInsertionControl(*vc, string2time(oc.getString("max-depart-delay")), oc.getBool("eager-insert"), oc.getInt("max-num-vehicles"));
	myVehicleControl = vc;
	myDetectorControl = new MSDetectorControl();
	myEdges = 0;
	myJunctions = 0;
	myRouteLoaders = 0;
	myLogics = 0;
	myPersonControl = 0;
	myContainerControl = 0;
	myEdgeWeights = 0;
	myShapeContainer = shapeCont == 0 ? new ShapeContainer() : shapeCont;

	myBeginOfTimestepEvents = beginOfTimestepEvents;
	myEndOfTimestepEvents = endOfTimestepEvents;
	myInsertionEvents = insertionEvents;
	myLanesRTree.first = false;

	if (MSGlobals::gUseMesoSim) {
		MSGlobals::gMesoNet = new MELoop(string2time(oc.getString("meso-recheck")));
	}
	myInstance = this;
}


bool MSNet::changeVehicleSpeed(std::string &firstVehicleName, std::string &secondVehcileName)
{
	//std::map<std::string, bool> vehiclePar;
	//vehiclePar.insert(pair<std::string, double>(firstVehicleName, false));
	//vehiclePar.insert(pair<std::string, double>(secondVehcileName, false));
	std::vector<vehiclePara>::iterator iter;
	bool m_firstVehilceFlag = false;
	bool m_secondVehilceFlag = false;
	for (iter = m_setSaveCurrentMaxSpeed.begin(); iter != m_setSaveCurrentMaxSpeed.end();iter++)
	{
		if (firstVehicleName == iter->vehicleName)
		{
			m_firstVehilceFlag = true;
		}
		if (secondVehcileName == iter->vehicleName)
		{
			m_secondVehilceFlag = true;
		}
	}


	std::cout << "Start" << " " << "vehicleName:" << firstVehicleName << " " <<
		" " << libsumo::Vehicle::getMaxSpeed(firstVehicleName) << " " <<
		"vehicleName:" << secondVehcileName <<
		" " << libsumo::Vehicle::getMaxSpeed(secondVehcileName) << std::endl;
	std::random_device rd;
	std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	std::uniform_real_distribution<> dis(0.1, 1.0);
	if (m_firstVehilceFlag == false)
	{
		vehiclePara m_vehiclePara;
		m_vehiclePara.vehicleName = firstVehicleName;
		m_vehiclePara.vehicleSpeed = libsumo::Vehicle::getMaxSpeed(firstVehicleName);
		m_setSaveCurrentMaxSpeed.push_back(m_vehiclePara);
		
	}
	if (m_secondVehilceFlag == false)
	{
		vehiclePara m_vehiclePara;
		m_vehiclePara.vehicleName = secondVehcileName;
		m_vehiclePara.vehicleSpeed = libsumo::Vehicle::getMaxSpeed(secondVehcileName);
		m_setSaveCurrentMaxSpeed.push_back(m_vehiclePara);
	}
	

	double m_dFirstVehcileRate = 0;
	double m_dSecondVehcileRate = 0;
	double m_dTheshold = 0.2;

	while (m_dSecondVehcileRate == m_dFirstVehcileRate || abs(m_dFirstVehcileRate - m_dSecondVehcileRate) < m_dTheshold)
	{
		m_dFirstVehcileRate = dis(gen);
		m_dSecondVehcileRate = dis(gen);
	}
	libsumo::Vehicle::setMaxSpeed(firstVehicleName, libsumo::Vehicle::getMaxSpeed(firstVehicleName)*m_dFirstVehcileRate);

	
	libsumo::Vehicle::setMaxSpeed(secondVehcileName, libsumo::Vehicle::getMaxSpeed(secondVehcileName)*m_dSecondVehcileRate);
	//double m_dFirstVehicleSpeed = libsumo::Vehicle::getSpeed(firstVehicleName);
	//double m_dSecondVehicleSpeed = libsumo::Vehicle::getSpeed(secondVehcileName);
	//std::cout << "Start" << " " << "vehicleName:" << firstVehicleName << " " << m_dFirstVehicleSpeed <<
	/*	" " << libsumo::Vehicle::getMaxSpeed(firstVehicleName) << " " <<
		"vehicleName:" << secondVehcileName << " " << m_dSecondVehicleSpeed <<
		" " << libsumo::Vehicle::getMaxSpeed(secondVehcileName) << std::endl;*/

	
	/*double m_dThreshold = 0.05;
	while (m_dSecondVehcileRate == m_dFirstVehcileRate || abs(m_dFirstVehcileRate - m_dSecondVehcileRate) < m_dThreshold)
	{
		
		m_dSecondVehcileRate = dis(gen);
	}*/
	//libsumo::Vehicle::setSpeed(firstVehicleName, m_dFirstVehcileRate*m_dFirstVehicleSpeed);
	//libsumo::Vehicle::setSpeed(secondVehcileName, m_dSecondVehcileRate*m_dSecondVehicleSpeed);

	

	std::cout << "End" << "   " << "vehicleName:" << firstVehicleName << " " << "" <<
		" " << libsumo::Vehicle::getMaxSpeed(firstVehicleName) << " " <<
		"vehicleName:" << secondVehcileName << " " << "" <<
		" " << libsumo::Vehicle::getMaxSpeed(secondVehcileName) << std::endl << std::endl;
	return true;
}


void
MSNet::closeBuilding(const OptionsCont& oc, MSEdgeControl* edges, MSJunctionControl* junctions,
	SUMORouteLoaderControl* routeLoaders,
	MSTLLogicControl* tlc,
	std::vector<SUMOTime> stateDumpTimes,
	std::vector<std::string> stateDumpFiles,
	bool hasInternalLinks,
	bool hasNeighs,
	bool lefthand,
	double version) {
	myEdges = edges;
	myJunctions = junctions;
	myRouteLoaders = routeLoaders;
	myLogics = tlc;
	// save the time the network state shall be saved at
	myStateDumpTimes = stateDumpTimes;
	myStateDumpFiles = stateDumpFiles;
	myStateDumpPeriod = string2time(oc.getString("save-state.period"));
	myStateDumpPrefix = oc.getString("save-state.prefix");
	myStateDumpSuffix = oc.getString("save-state.suffix");

	// set requests/responses
	myJunctions->postloadInitContainer();

	// initialise performance computation
	if (myLogExecutionTime) {
		mySimBeginMillis = SysUtils::getCurrentMillis();
	}
	myHasInternalLinks = hasInternalLinks;
	if (hasNeighs && MSGlobals::gLateralResolution > 0) {
		WRITE_WARNING("Opposite direction driving does not work together with the sublane model.");
	}
	myHasElevation = checkElevation();
	myLefthand = lefthand;
	myVersion = version;
}


MSNet::~MSNet() {
	// delete events first maybe they do some cleanup
	delete myBeginOfTimestepEvents;
	myBeginOfTimestepEvents = 0;
	delete myEndOfTimestepEvents;
	myEndOfTimestepEvents = 0;
	delete myInsertionEvents;
	myInsertionEvents = 0;
	// delete controls
	delete myJunctions;
	delete myDetectorControl;
	// delete mean data
	delete myEdges;
	delete myInserter;
	delete myLogics;
	delete myRouteLoaders;
	if (myPersonControl != 0) {
		delete myPersonControl;
	}
	if (myContainerControl != 0) {
		delete myContainerControl;
	}
	delete myVehicleControl; // must happen after deleting transportables
	delete myShapeContainer;
	delete myEdgeWeights;
	delete myRouterTT;
	delete myRouterEffort;
	if (myPedestrianRouter != 0) {
		delete myPedestrianRouter;
	}
	myLanesRTree.second.RemoveAll();
	clearAll();
	if (MSGlobals::gUseMesoSim) {
		delete MSGlobals::gMesoNet;
	}
	//    delete myPedestrianRouter;
	myInstance = 0;
}


void
MSNet::addRestriction(const std::string& id, const SUMOVehicleClass svc, const double speed) {
	myRestrictions[id][svc] = speed;
}


const std::map<SUMOVehicleClass, double>*
MSNet::getRestrictions(const std::string& id) const {
	std::map<std::string, std::map<SUMOVehicleClass, double> >::const_iterator i = myRestrictions.find(id);
	if (i == myRestrictions.end()) {
		return 0;
	}
	return &i->second;
}


MSNet::SimulationState
MSNet::simulate(SUMOTime start, SUMOTime stop) {
	// report the begin when wished
	WRITE_MESSAGE("Simulation started with time: " + time2string(start));
	// the simulation loop
	SimulationState state = SIMSTATE_RUNNING;
	// state loading may have changed the start time so we need to reinit it
	myStep = start;
#ifndef NO_TRACI
#ifdef HAVE_PYTHON
	if (OptionsCont::getOptions().isSet("python-script")) {
		TraCIServer::runEmbedded(OptionsCont::getOptions().getString("python-script"));
		closeSimulation(start);
		WRITE_MESSAGE("Simulation ended at time: " + time2string(getCurrentTimeStep()));
		WRITE_MESSAGE("Reason: Script ended");
		return state;
	}
#endif
#endif
	while (state == SIMSTATE_RUNNING) {
		if (myLogStepNumber) {
			preSimStepOutput();
		}
		simulationStep();
		if (myLogStepNumber) {
			postSimStepOutput();
		}
		state = simulationState(stop);
#ifdef DEBUG_SIMSTEP
		std::cout << SIMTIME << " MSNet::simulate(" << start << ", " << stop << ")"
			<< "\n simulation state: " << getStateMessage(state)
			<< std::endl;
#endif
#ifndef NO_TRACI
		if (state == SIMSTATE_LOADING) {
			OptionsIO::setArgs(TraCIServer::getInstance()->getLoadArgs());
			TraCIServer::getInstance()->getLoadArgs().clear();
		}
		else if (state != SIMSTATE_RUNNING) {
			if (TraCIServer::getInstance() != 0 && !TraCIServer::wasClosed()) {
				// overrides SIMSTATE_END_STEP_REACHED, e.g. (TraCI ignore SUMO's --end option)
				state = SIMSTATE_RUNNING;
			}
		}
#endif
	}
	// report the end when wished
	WRITE_MESSAGE("Simulation ended at time: " + time2string(getCurrentTimeStep()));
	WRITE_MESSAGE("Reason: " + getStateMessage(state));
	// exit simulation loop
	closeSimulation(start);
	return state;
}

void
MSNet::loadRoutes() {
	myRouteLoaders->loadNext(myStep);
}


void
MSNet::closeSimulation(SUMOTime start) {
	myDetectorControl->close(myStep);
	if (OptionsCont::getOptions().getBool("vehroute-output.write-unfinished")) {
		MSDevice_Vehroutes::generateOutputForUnfinished();
	}
	if (OptionsCont::getOptions().getBool("tripinfo-output.write-unfinished")) {
		MSDevice_Tripinfo::generateOutputForUnfinished();
	}
	if (OptionsCont::getOptions().isSet("chargingstations-output")) {
		writeChargingStationOutput();
	}
	if (myLogExecutionTime) {
		long duration = SysUtils::getCurrentMillis() - mySimBeginMillis;
		std::ostringstream msg;
		// print performance notice
		msg << "Performance: " << "\n" << " Duration: " << duration << "ms" << "\n";
		if (duration != 0) {
			msg << " Real time factor: " << (STEPS2TIME(myStep - start) * 1000. / (double)duration) << "\n";
			msg.setf(std::ios::fixed, std::ios::floatfield);    // use decimal format
			msg.setf(std::ios::showpoint);    // print decimal point
			msg << " UPS: " << ((double)myVehiclesMoved / ((double)duration / 1000)) << "\n";
		}
		// print vehicle statistics
		const std::string discardNotice = ((myVehicleControl->getLoadedVehicleNo() != myVehicleControl->getDepartedVehicleNo()) ?
			" (Loaded: " + toString(myVehicleControl->getLoadedVehicleNo()) + ")" : "");
		msg << "Vehicles: " << "\n"
			<< " Inserted: " << myVehicleControl->getDepartedVehicleNo() << discardNotice << "\n"
			<< " Running: " << myVehicleControl->getRunningVehicleNo() << "\n"
			<< " Waiting: " << myInserter->getWaitingVehicleNo() << "\n";

		if (myVehicleControl->getTeleportCount() > 0 || myVehicleControl->getCollisionCount() > 0) {
			// print optional teleport statistics
			std::vector<std::string> reasons;
			if (myVehicleControl->getCollisionCount() > 0) {
				reasons.push_back("Collisions: " + toString(myVehicleControl->getCollisionCount()));
			}
			if (myVehicleControl->getTeleportsJam() > 0) {
				reasons.push_back("Jam: " + toString(myVehicleControl->getTeleportsJam()));
			}
			if (myVehicleControl->getTeleportsYield() > 0) {
				reasons.push_back("Yield: " + toString(myVehicleControl->getTeleportsYield()));
			}
			if (myVehicleControl->getTeleportsWrongLane() > 0) {
				reasons.push_back("Wrong Lane: " + toString(myVehicleControl->getTeleportsWrongLane()));
			}
			msg << "Teleports: " << myVehicleControl->getTeleportCount() << " (" << joinToString(reasons, ", ") << ")\n";
		}
		if (myVehicleControl->getEmergencyStops() > 0) {
			msg << "Emergency Stops: " << myVehicleControl->getEmergencyStops() << "\n";
		}
		if (myPersonControl != 0 && myPersonControl->getLoadedNumber() > 0) {
			msg << "Persons: " << "\n"
				<< " Inserted: " << myPersonControl->getLoadedNumber() << "\n"
				<< " Running: " << myPersonControl->getRunningNumber() << "\n";
			if (myPersonControl->getJammedNumber() > 0) {
				msg << " Jammed: " << myPersonControl->getJammedNumber() << "\n";
			}
		}
		if (OptionsCont::getOptions().getBool("duration-log.statistics")) {
			msg << MSDevice_Tripinfo::printStatistics();
		}
		WRITE_MESSAGE(msg.str());
	}
}

#include <time.h>
void
MSNet::simulationStep() {

#ifdef DEBUG_SIMSTEP
	std::cout << SIMTIME << ": MSNet::simulationStep() called"
		<< ", myStep = " << myStep
		<< std::endl;
#endif
#ifndef NO_TRACI
	if (myLogExecutionTime) {
		myTraCIStepDuration = SysUtils::getCurrentMillis();
	}
	TraCIServer* t = TraCIServer::getInstance();
	if (t != 0 && !t->isEmbedded()) {
		t->processCommandsUntilSimStep(myStep);
#ifdef DEBUG_SIMSTEP
		bool loadRequested = !TraCI::getLoadArgs().empty();
		bool closed = TraCIServer::wasClosed();
		assert(t->getTargetTime() >= myStep || loadRequested || closed);
#endif
	}
	if (myLogExecutionTime) {
		myTraCIStepDuration = SysUtils::getCurrentMillis() - myTraCIStepDuration;
	}
#ifdef DEBUG_SIMSTEP
	std::cout << SIMTIME << ": TraCI target time: " << t->getTargetTime() << std::endl;
#endif
#endif
	// execute beginOfTimestepEvents
	if (myLogExecutionTime) {
		mySimStepDuration = SysUtils::getCurrentMillis();
	}
	// simulation state output
	std::vector<SUMOTime>::iterator timeIt = find(myStateDumpTimes.begin(), myStateDumpTimes.end(), myStep);
	if (timeIt != myStateDumpTimes.end()) {
		const int dist = (int)distance(myStateDumpTimes.begin(), timeIt);
		MSStateHandler::saveState(myStateDumpFiles[dist], myStep);
	}
	if (myStateDumpPeriod > 0 && myStep % myStateDumpPeriod == 0) {
		MSStateHandler::saveState(myStateDumpPrefix + "_" + time2string(myStep) + myStateDumpSuffix, myStep);
	}
	myBeginOfTimestepEvents->execute(myStep);
#ifdef HAVE_FOX
	MSDevice_Routing::waitForAll();
#endif
	if (MSGlobals::gCheck4Accidents) {
		myEdges->detectCollisions(myStep, STAGE_EVENTS);
	}
	// check whether the tls programs need to be switched
	myLogics->check2Switch(myStep);

	if (MSGlobals::gUseMesoSim) {
		MSGlobals::gMesoNet->simulate(myStep);
	}
	else {
		// assure all lanes with vehicles are 'active'
		myEdges->patchActiveLanes();

		// compute safe velocities for all vehicles for the next few lanes
		// also register ApproachingVehicleInformation for all links
		myEdges->planMovements(myStep);

		// decide right-of-way and execute movements
		myEdges->executeMovements(myStep);
		if (MSGlobals::gCheck4Accidents) {
			myEdges->detectCollisions(myStep, STAGE_MOVEMENTS);
		}

		// vehicles may change lanes
		myEdges->changeLanes(myStep);

		if (MSGlobals::gCheck4Accidents) {
			myEdges->detectCollisions(myStep, STAGE_LANECHANGE);
		}
	}
	loadRoutes();

	// persons
	if (myPersonControl != 0 && myPersonControl->hasTransportables()) {
		myPersonControl->checkWaiting(this, myStep);
	}
	// containers
	if (myContainerControl != 0 && myContainerControl->hasTransportables()) {
		myContainerControl->checkWaiting(this, myStep);
	}
	// insert vehicles
	myInserter->determineCandidates(myStep);
	myInsertionEvents->execute(myStep);
#ifdef HAVE_FOX
	MSDevice_Routing::waitForAll();
#endif
	myInserter->emitVehicles(myStep);
	if (MSGlobals::gCheck4Accidents) {
		//myEdges->patchActiveLanes(); // @note required to detect collisions on lanes that were empty before insertion. wasteful?
		myEdges->detectCollisions(myStep, STAGE_INSERTIONS);
	}
	MSVehicleTransfer::getInstance()->checkInsertions(myStep);
	MSVehicleTransfer::getInstance()->processBlocking();

	// execute endOfTimestepEvents
	myEndOfTimestepEvents->execute(myStep);

#ifndef NO_TRACI
	if (TraCIServer::getInstance() != 0) {
		if (myLogExecutionTime) {
			myTraCIStepDuration -= SysUtils::getCurrentMillis();
		}
		libsumo::Helper::postProcessRemoteControl();
		if (myLogExecutionTime) {
			myTraCIStepDuration += SysUtils::getCurrentMillis();
		}
	}
#endif
	// update and write (if needed) detector values
	writeOutput();

	if (myLogExecutionTime) {
		mySimStepDuration = SysUtils::getCurrentMillis() - mySimStepDuration;
		myVehiclesMoved += myVehicleControl->getRunningVehicleNo();
	}
	myStep += DELTA_T;

	clock_t m_time = clock();
	Position position(890.15, 764.43, 18.67);
	PositionVector shape;
	shape.push_back(position);
	/*shape.push_back(Position(551.67, 1006.46, 18.29));
	shape.push_back(Position(1123.24, 529.38, 19.40));*/
	double range = 100;
	std::set<std::string> info;
	libsumo::Helper::collectObjectsInRange(CMD_GET_VEHICLE_VARIABLE, shape, range, info);

	if (info.size() <= 0)
	{
		return;
	}


	std::vector<std::string> vehicleName;
	std::set <std::string > ::iterator iter;
	std::vector<std::vector<collisionDetectedPosition>> m_vecVehcilePrePositon;



	for (iter = info.begin(); iter != info.end(); iter++)
	{
		vehicleName.push_back(*iter);
		std::vector<libsumo::TraCIPosition> trace = libsumo::Vehicle::getPredictTrace(*iter);
		if (trace.size() <= 0)
		{
			continue;
		}
		std::vector<collisionDetectedPosition>  m_vecCurrentVehiclPrePosition(trace.size());
		for (int i = 0; i < trace.size(); i++)
		{

			m_vecCurrentVehiclPrePosition[i].x = trace[i].x;
			m_vecCurrentVehiclPrePosition[i].y = trace[i].y;
			m_vecCurrentVehiclPrePosition[i].z = trace[i].z;
			m_vecCurrentVehiclPrePosition[i].speed = libsumo::Vehicle::getSpeed(*iter);
			m_vecCurrentVehiclPrePosition[i].angle = libsumo::Vehicle::getAngle(*iter);
			m_vecCurrentVehiclPrePosition[i].width = libsumo::Vehicle::getWidth(*iter);
			m_vecCurrentVehiclPrePosition[i].length = libsumo::Vehicle::getLength(*iter);
		}
		m_vecVehcilePrePositon.push_back(m_vecCurrentVehiclPrePosition);
		m_vecCurrentVehiclPrePosition.clear();
	}

	std::vector<vehiclePara>::iterator  m_iterator;
	if (m_setSaveCurrentMaxSpeed.size() > 1e+3)
	{
		m_setSaveCurrentMaxSpeed.clear();
	}
	if (!m_setSaveCurrentMaxSpeed.empty())
	{
		for (m_iterator = m_setSaveCurrentMaxSpeed.begin(); m_iterator != m_setSaveCurrentMaxSpeed.end(); )
		{
			bool m_bFlag = false;
			for (iter = info.begin(); iter != info.end(); iter++)
			{
				if (*iter == m_iterator->vehicleName)
				{
					m_bFlag = true;
				}
			}
			if (m_bFlag == false)
			{
				libsumo::Vehicle::setMaxSpeed(m_iterator->vehicleName, m_iterator->vehicleSpeed);
				m_iterator = m_setSaveCurrentMaxSpeed.erase(m_iterator);

			}
			else if (m_bFlag == true)
			{
				m_iterator++;
			}
		}
	}

	unsigned char LevelCoefficient = 4;
	if (LevelCoefficient <= 1 || vehicleName.size() <= 0)
	{
		return;
	}



	for (int i = 0; i < vehicleName.size(); i++)
	{

		unsigned int m_uIFirstVehiclePositionSize = m_vecVehcilePrePositon[i].size() - 1;
		if (m_uIFirstVehiclePositionSize < 0)
		{
			continue;
		}
		collisionDetected *m_firstCollisionDetected = new collisionDetected(/*LevelCoefficient,*/ m_vecVehcilePrePositon[i]);
		bool m_firstTraceIsJudgePositionYChange = false;
		m_firstTraceIsJudgePositionYChange = m_firstCollisionDetected->IsTwoCoordinateConsistent(m_vecVehcilePrePositon[i], "col");
		bool m_firstTraceIsJudgePositionXChange = false;
		m_firstTraceIsJudgePositionXChange = m_firstCollisionDetected->IsTwoCoordinateConsistent(m_vecVehcilePrePositon[i], "row");

		for (int j = i + 1; j < vehicleName.size(); j++)
		{
			unsigned int m_uISecondVehiclePositionSize = m_vecVehcilePrePositon[j].size() - 1;
			if (m_uISecondVehiclePositionSize < 0)
			{

				continue;
			}
			collisionDetected *m_secondCollisionDetected = new collisionDetected(/*LevelCoefficient,*/ m_vecVehcilePrePositon[j]);
			bool m_secondTraceIsJudgePositionYChange = false;
			m_secondTraceIsJudgePositionYChange = m_secondCollisionDetected->IsTwoCoordinateConsistent(m_vecVehcilePrePositon[j], "col");


			double m_fMaxVehicleDistance = (m_vecVehcilePrePositon[i][0].length > m_vecVehcilePrePositon[j][0].length) ? m_vecVehcilePrePositon[i][0].length : m_vecVehcilePrePositon[j][0].length;
			m_fMaxVehicleDistance = m_fMaxVehicleDistance * 2;
			if (m_firstTraceIsJudgePositionYChange == false && m_secondTraceIsJudgePositionYChange == false
				&& abs(m_vecVehcilePrePositon[i][0].x - m_vecVehcilePrePositon[j][0].x) <= m_fMaxVehicleDistance)//??????§Ò????????????????????????
			{
				double m_initialPositionY = m_vecVehcilePrePositon[i][0].y - m_vecVehcilePrePositon[j][0].y;
				double m_finalPositionY = m_vecVehcilePrePositon[i][m_uIFirstVehiclePositionSize].y -
					m_vecVehcilePrePositon[j][m_uISecondVehiclePositionSize].y;

				if (m_initialPositionY*m_finalPositionY <= 0)
				{
					changeVehicleSpeed(vehicleName[i], vehicleName[j]);
					m_secondCollisionDetected = NULL;
					delete[] m_secondCollisionDetected;
					continue;
				}
			}


			bool m_secondTraceIsJudgePositionXChange = false;
			m_secondTraceIsJudgePositionXChange = m_secondCollisionDetected->IsTwoCoordinateConsistent(m_vecVehcilePrePositon[j], "row");

			if (m_firstTraceIsJudgePositionXChange == false && m_firstTraceIsJudgePositionXChange == false
				&& abs(m_vecVehcilePrePositon[i][0].y - m_vecVehcilePrePositon[j][0].y) <= m_fMaxVehicleDistance)//??????§Ò????????????????????????
			{
				double m_initialPositionX = m_vecVehcilePrePositon[i][0].x - m_vecVehcilePrePositon[j][0].x;
				double m_finalPositionX = m_vecVehcilePrePositon[i][m_uIFirstVehiclePositionSize].x
					- m_vecVehcilePrePositon[j][m_uISecondVehiclePositionSize].x;
				if (m_initialPositionX*m_finalPositionX <= 0)
				{
					changeVehicleSpeed(vehicleName[i], vehicleName[j]);
					m_secondCollisionDetected = NULL;
					delete[] m_secondCollisionDetected;
					continue;
				}

			}


			double m_fFirstVehiclePolynomialCoefficient[10] = { 0 }; //?????????????????
			double m_fSecondVehiclePolynomialCoefficient[10] = { 0 }; //?????????????????
			double m_fPolynomialCoefficient[10] = { 0 }; //?????????????????

			if (m_firstTraceIsJudgePositionYChange == false && m_secondTraceIsJudgePositionYChange == true)//consdier that position which has been sorted,where the min position may be the first position
			{
				if (m_vecVehcilePrePositon[i][0].x >= m_vecVehcilePrePositon[j][0].x
					&&m_vecVehcilePrePositon[i][m_uIFirstVehiclePositionSize].x <= m_vecVehcilePrePositon[j][m_uISecondVehiclePositionSize].x
					/*&&abs(m_vecVehcilePrePositon[i][0].y-m_vecVehcilePrePositon[j][0].x)<=(m_vecVehcilePrePositon[i][0].width+m_vecVehcilePrePositon[j][0].width)/2.0*/)
				{
					changeVehicleSpeed(vehicleName[i], vehicleName[j]);
					m_secondCollisionDetected = NULL;
					delete[] m_secondCollisionDetected;
					continue;
				}
				else
				{
					std::vector<vehiclePara>::iterator m_vecVehiclePar;
					for (m_vecVehiclePar = m_setSaveCurrentMaxSpeed.begin(); m_vecVehiclePar != m_setSaveCurrentMaxSpeed.end();)
					{
						if (m_vecVehiclePar->vehicleName == vehicleName[j]|| m_vecVehiclePar->vehicleName == vehicleName[i])
						{
							libsumo::Vehicle::setMaxSpeed(m_vecVehiclePar->vehicleName, m_vecVehiclePar->vehicleSpeed);
							m_vecVehiclePar = m_setSaveCurrentMaxSpeed.erase(m_vecVehiclePar);
						}
						else
						{
							m_vecVehiclePar++;
						}
					}
				}
			}
			else if (m_firstTraceIsJudgePositionYChange == true && m_secondTraceIsJudgePositionYChange == false)
			{
				if (m_vecVehcilePrePositon[j][0].x >= m_vecVehcilePrePositon[i][0].x
					&&m_vecVehcilePrePositon[j][m_uISecondVehiclePositionSize].x <= m_vecVehcilePrePositon[i][m_uIFirstVehiclePositionSize].x
					/*&&abs(m_vecVehcilePrePositon[i][0].y - m_vecVehcilePrePositon[j][0].x) <= (m_vecVehcilePrePositon[i][0].width + m_vecVehcilePrePositon[j][0].width) / 2.0*/)
				{
					changeVehicleSpeed(vehicleName[i], vehicleName[j]);
					m_secondCollisionDetected = NULL;
					delete[] m_secondCollisionDetected;
					continue;
				}
				else
				{
					std::vector<vehiclePara>::iterator m_vecVehiclePar;
					for (m_vecVehiclePar = m_setSaveCurrentMaxSpeed.begin(); m_vecVehiclePar != m_setSaveCurrentMaxSpeed.end();)
					{
						if (m_vecVehiclePar->vehicleName == vehicleName[j] || m_vecVehiclePar->vehicleName == vehicleName[i])
						{
							libsumo::Vehicle::setMaxSpeed(m_vecVehiclePar->vehicleName, m_vecVehiclePar->vehicleSpeed);
							m_vecVehiclePar = m_setSaveCurrentMaxSpeed.erase(m_vecVehiclePar);
						}
						else
						{
							m_vecVehiclePar++;
						}
					}
				}

			}
			else if (m_firstTraceIsJudgePositionXChange == true && m_firstTraceIsJudgePositionYChange == true &&
				m_secondTraceIsJudgePositionXChange == true && m_secondTraceIsJudgePositionYChange == true)
			{
				std::vector<collisionDetectedPosition> m_cmpVehiclePosition(m_uIFirstVehiclePositionSize + 1);
				if (m_vecVehcilePrePositon[i][0].speed < 1.0)
				{
					m_fFirstVehiclePolynomialCoefficient[1] = -1.0*tan(m_vecVehcilePrePositon[i][0].angle)*m_vecVehcilePrePositon[i][0].x+ m_vecVehcilePrePositon[i][0].y;
					m_fFirstVehiclePolynomialCoefficient[2] = tan(m_vecVehcilePrePositon[i][0].angle);
				}
				else
				{
					m_firstCollisionDetected->AugmentedMatrixAssignment((int)m_uIFirstVehiclePositionSize, LevelCoefficient, m_fFirstVehiclePolynomialCoefficient);
				}
				if (m_vecVehcilePrePositon[j][0].speed < 1.0)
				{
					m_fSecondVehiclePolynomialCoefficient[1] = -1.0*tan(m_vecVehcilePrePositon[j][0].angle)*m_vecVehcilePrePositon[j][0].x + m_vecVehcilePrePositon[j][0].y;
					m_fSecondVehiclePolynomialCoefficient[2] = tan(m_vecVehcilePrePositon[j][0].angle);
				}
				else
				{
					m_secondCollisionDetected->AugmentedMatrixAssignment((int)m_uIFirstVehiclePositionSize, LevelCoefficient, m_fSecondVehiclePolynomialCoefficient);
				}
				collisionDetected *m_cmpCollisionDetected = new collisionDetected(/*LevelCoefficient,*/ m_cmpVehiclePosition);
				m_cmpVehiclePosition.clear();

				for (int k = 0; k < m_uIFirstVehiclePositionSize; k++)
				{
					m_fPolynomialCoefficient[k] = m_fFirstVehiclePolynomialCoefficient[k] - m_fSecondVehiclePolynomialCoefficient[k];
				}

				double m_fResult[3] = { 0 };
				unsigned char m_iEquationSolutionNum = 0;
				bool m_bMeetPointFlag = m_cmpCollisionDetected->EquationSolution(LevelCoefficient, m_fPolynomialCoefficient, m_iEquationSolutionNum, m_fResult);//?????????????¨®????????????????????????????

				m_cmpCollisionDetected = NULL;
				delete[] m_cmpCollisionDetected;

				if (m_bMeetPointFlag == true && m_iEquationSolutionNum > 0)
				{
					//double m_fBias = 2;//consider the vehicle width and height which cases the time cost
					for (unsigned char m_cEquationSolutionNum = 0; m_cEquationSolutionNum < (unsigned char)m_iEquationSolutionNum; m_cEquationSolutionNum++)
					{

						double m_fFirstVehicleMinPosition = (m_vecVehcilePrePositon[i][0].x < m_vecVehcilePrePositon[i][m_uIFirstVehiclePositionSize].x) ?
							m_vecVehcilePrePositon[i][0].x : m_vecVehcilePrePositon[i][m_uIFirstVehiclePositionSize].x;
						double m_fFirstVehicleMaxPosition = (m_vecVehcilePrePositon[i][0].x > m_vecVehcilePrePositon[i][m_uIFirstVehiclePositionSize].x) ?
							m_vecVehcilePrePositon[i][0].x : m_vecVehcilePrePositon[i][m_uIFirstVehiclePositionSize].x;
						double m_fSecondVehicleMinPosition = (m_vecVehcilePrePositon[j][0].x < m_vecVehcilePrePositon[j][m_uISecondVehiclePositionSize].x) ?
							m_vecVehcilePrePositon[j][0].x : m_vecVehcilePrePositon[j][m_uISecondVehiclePositionSize].x;
						double m_fSecondVehicleMaxPosition = (m_vecVehcilePrePositon[j][0].x > m_vecVehcilePrePositon[j][m_uISecondVehiclePositionSize].x) ?
							m_vecVehcilePrePositon[j][0].x : m_vecVehcilePrePositon[j][m_uISecondVehiclePositionSize].x;


						double m_fMinPosition = (m_fFirstVehicleMinPosition > m_fSecondVehicleMinPosition) ? m_fFirstVehicleMinPosition : m_fSecondVehicleMinPosition;
						double m_fMaxPosition = (m_fFirstVehicleMaxPosition < m_fSecondVehicleMaxPosition) ? m_fSecondVehicleMaxPosition : m_fSecondVehicleMaxPosition;
						
						m_fMinPosition = m_fMinPosition - m_fMaxVehicleDistance;
						m_fMaxPosition = m_fMaxPosition + m_fMaxVehicleDistance;
						if (m_fMinPosition <= m_fResult[m_cEquationSolutionNum] && m_fMaxPosition >= m_fResult[m_cEquationSolutionNum])
						{

							double m_firstVehicleTime = 0;
							double m_secondVehicleTime = 0;

							double m_firstVehilceDistance = 0;
							double m_secondVehilceDistance = 0;
							m_firstVehilceDistance = abs(m_firstCollisionDetected->GetLineLength(LevelCoefficient, m_fFirstVehiclePolynomialCoefficient,
								m_vecVehcilePrePositon[i][0].x, m_fResult[m_cEquationSolutionNum]));
							m_secondVehilceDistance = abs(m_secondCollisionDetected->GetLineLength(LevelCoefficient, m_fSecondVehiclePolynomialCoefficient,
								m_vecVehcilePrePositon[j][0].x, m_fResult[m_cEquationSolutionNum]));


							//0+a+bx+cx2
							double m_firstVehilceCoefficient[4] = { 0,m_firstVehilceDistance,-1.0*m_vecVehcilePrePositon[i][0].speed,0.5*libsumo::Vehicle::getAccel(vehicleName[i]) };
							double m_secondVehilceCoefficient[4] = { 0,m_secondVehilceDistance,-1.0*m_vecVehcilePrePositon[j][0].speed,0.5*libsumo::Vehicle::getAccel(vehicleName[j]) };

							unsigned char m_cCoefficientNum = 3;
							/*m_firstVehicleTime = abs(m_firstCollisionDetected->GetLineLength(LevelCoefficient, m_fFirstVehiclePolynomialCoefficient,
								m_vecVehcilePrePositon[i][0].x, m_fResult[i])) / m_vecVehcilePrePositon[i][0].speed;

							m_secondVehicleTime = abs(m_secondCollisionDetected->GetLineLength(LevelCoefficient, m_fSecondVehiclePolynomialCoefficient,
								m_vecVehcilePrePositon[j][0].x, m_fResult[i])) / m_vecVehcilePrePositon[j][0].speed;*/
								//	}
							unsigned char m_cFirstVehilceEquationSolutionNum = 0;
							double m_firstVehilceTimeResult[2] = { 0 };
							bool m_bFirstVehilce = false;
							m_bFirstVehilce = m_firstCollisionDetected->UnivariateQuadraticSolution(m_cCoefficientNum, m_firstVehilceCoefficient, m_cFirstVehilceEquationSolutionNum, m_firstVehilceTimeResult);
							if (m_bFirstVehilce == false)
							{
								continue;
							}
							for (unsigned char num = 0; num < 2; num++)
							{
								if (m_firstVehilceTimeResult[num] <= 0)
								{
									m_bFirstVehilce = false;
									continue;
								}
								else
								{
									m_bFirstVehilce = true;
									m_firstVehicleTime = m_firstVehilceTimeResult[num];
									break;
								}
							}
							if (m_bFirstVehilce == false)
							{
								continue;
							}

							unsigned char m_cSecondVehilceEquationSolutionNum = 0;
							double m_secondVehilceTimeResult[2] = { 0 };
							bool m_bSecondVehilce = false;
							m_bSecondVehilce=m_secondCollisionDetected->UnivariateQuadraticSolution(m_cCoefficientNum, m_secondVehilceCoefficient, m_cSecondVehilceEquationSolutionNum, m_secondVehilceTimeResult);
							
							if (m_bSecondVehilce == false)
							{
								continue;
							}
							for (unsigned char num = 0; num < 2; num++)
							{
								if (m_secondVehilceTimeResult[num] <= 0)
								{
									m_bSecondVehilce = false;
									continue;
								}
								else
								{
									m_bSecondVehilce = true;
									m_secondVehicleTime = m_secondVehilceTimeResult[num];
									break;
								}
							}
							if (m_bSecondVehilce == false)
							{
								continue;
							}


							//}
							double m_fTimeBias = abs(m_firstVehicleTime - m_secondVehicleTime);
							if (m_fTimeBias == 0)
							{
								changeVehicleSpeed(vehicleName[i], vehicleName[j]);
								/*	m_secondCollisionDetected = NULL;
									delete[] m_secondCollisionDetected;
									continue;*/
									//std::cout << "collision" << " " << m_fMinPosition << " " << m_fMaxPosition << std::endl;

							}
							else if (m_fTimeBias > 0)
							{
								double  m_firstCofficientDerivation = 0;
								bool m_firstDerivationFlag = false;
								double  m_secondCofficientDerivation = 0;
								bool m_secondDerivationFlag = false;


								m_firstDerivationFlag = m_firstCollisionDetected->Derivation(LevelCoefficient,
									m_fFirstVehiclePolynomialCoefficient, m_fResult[i], m_firstCofficientDerivation);

								m_secondDerivationFlag = m_firstCollisionDetected->Derivation(LevelCoefficient,
									m_fSecondVehiclePolynomialCoefficient, m_fResult[i], m_secondCofficientDerivation);

								if (m_firstDerivationFlag == true && m_secondDerivationFlag == true)
								{
									double  m_fTheta = abs(atan(m_firstCofficientDerivation - m_secondCofficientDerivation));
									if (m_firstVehicleTime < m_secondVehicleTime)
									{
										double m_fDistanceBias = m_vecVehcilePrePositon[i][0].speed*sin(m_fTheta)*m_fTimeBias;
										if (m_fDistanceBias >= (m_vecVehcilePrePositon[i][0].length + m_vecVehcilePrePositon[j][0].width) / 2)
										{
											std::vector<vehiclePara>::iterator m_vecVehiclePar;
											for (m_vecVehiclePar = m_setSaveCurrentMaxSpeed.begin(); m_vecVehiclePar != m_setSaveCurrentMaxSpeed.end();)
											{
												if (m_vecVehiclePar->vehicleName == vehicleName[j] || m_vecVehiclePar->vehicleName == vehicleName[i])
												{
													libsumo::Vehicle::setMaxSpeed(m_vecVehiclePar->vehicleName, m_vecVehiclePar->vehicleSpeed);
													m_vecVehiclePar = m_setSaveCurrentMaxSpeed.erase(m_vecVehiclePar);
												}
												else
												{
													m_vecVehiclePar++;
												}
											}
										}
										else
										{
											changeVehicleSpeed(vehicleName[i], vehicleName[j]);
											/*					m_secondCollisionDetected = NULL;
																delete[] m_secondCollisionDetected;
																continue;*/
																//break;
										}
									}
									else
									{
										double m_fDistanceBias = m_vecVehcilePrePositon[j][0].speed*sin(m_fTheta)*m_fTimeBias;
										if (m_fDistanceBias >= (m_vecVehcilePrePositon[j][0].length + m_vecVehcilePrePositon[i][0].width) / 2)
										{
											std::vector<vehiclePara>::iterator m_vecVehiclePar;
											for (m_vecVehiclePar = m_setSaveCurrentMaxSpeed.begin(); m_vecVehiclePar != m_setSaveCurrentMaxSpeed.end();)
											{
												if (m_vecVehiclePar->vehicleName == vehicleName[j] || m_vecVehiclePar->vehicleName == vehicleName[i])
												{
													libsumo::Vehicle::setMaxSpeed(m_vecVehiclePar->vehicleName, m_vecVehiclePar->vehicleSpeed);
													m_vecVehiclePar = m_setSaveCurrentMaxSpeed.erase(m_vecVehiclePar);
												}
												else
												{
													m_vecVehiclePar++;
												}
											}
										}
										else
										{
											changeVehicleSpeed(vehicleName[i], vehicleName[j]);
									
										}
									}
								}

							}
						}
					}
				}
			}
			m_secondCollisionDetected = NULL;
			delete[] m_secondCollisionDetected;
		}
		m_firstCollisionDetected = NULL;
		delete[]m_firstCollisionDetected;
	}
	std::cout << "run time:" << clock() - m_time << "ms" << std::endl;
}


MSNet::SimulationState
MSNet::simulationState(SUMOTime stopTime) const {
#ifndef NO_TRACI
	if (TraCIServer::wasClosed()) {
		return SIMSTATE_CONNECTION_CLOSED;
	}
	if (TraCIServer::getInstance() != 0 && !TraCIServer::getInstance()->getLoadArgs().empty()) {
		return SIMSTATE_LOADING;
	}
	if ((stopTime < 0 || myStep > stopTime) && TraCIServer::getInstance() == 0) {
#else
	if (stopTime < 0 || myStep > stopTime) {
#endif
		if (myInsertionEvents->isEmpty()
			&& (myVehicleControl->getActiveVehicleCount() == 0)
			&& (myInserter->getPendingFlowCount() == 0)
			&& (myPersonControl == 0 || !myPersonControl->hasNonWaiting())
			&& (myContainerControl == 0 || !myContainerControl->hasNonWaiting())) {
			if (myPersonControl) {
				myPersonControl->abortWaitingForVehicle();
			}
			if (myContainerControl) {
				myContainerControl->abortWaitingForVehicle();
			}
			myVehicleControl->abortWaiting();
			return SIMSTATE_NO_FURTHER_VEHICLES;
		}
	}
	if (stopTime >= 0 && myStep >= stopTime) {
		return SIMSTATE_END_STEP_REACHED;
	}
	if (myMaxTeleports >= 0 && myVehicleControl->getTeleportCount() > myMaxTeleports) {
		return SIMSTATE_TOO_MANY_TELEPORTS;
	}
	return SIMSTATE_RUNNING;
	}


std::string
MSNet::getStateMessage(MSNet::SimulationState state) {
	switch (state) {
	case MSNet::SIMSTATE_RUNNING:
		return "";
	case MSNet::SIMSTATE_END_STEP_REACHED:
		return "The final simulation step has been reached.";
	case MSNet::SIMSTATE_NO_FURTHER_VEHICLES:
		return "All vehicles have left the simulation.";
	case MSNet::SIMSTATE_CONNECTION_CLOSED:
		return "TraCI requested termination.";
	case MSNet::SIMSTATE_ERROR_IN_SIM:
		return "An error occured (see log).";
	case MSNet::SIMSTATE_TOO_MANY_TELEPORTS:
		return "Too many teleports.";
	case MSNet::SIMSTATE_LOADING:
		return "TraCI issued load command.";
	default:
		return "Unknown reason.";
	}
}


void
MSNet::clearAll() {
	// clear container
	MSEdge::clear();
	MSLane::clear();
	MSRoute::clear();
	delete MSVehicleTransfer::getInstance();
	MSDevice::cleanupAll();
	MSTrigger::cleanup();
	MSCalibrator::cleanup();
	MSPModel::cleanup();
	MSCModel_NonInteracting::cleanup();
	MSDevice_BTsender::cleanup();
	MSDevice_SSM::cleanup();
#ifndef NO_TRACI
	TraCIServer* t = TraCIServer::getInstance();
	if (t != 0) {
		t->cleanup();
	}
	libsumo::Helper::cleanup();
#endif
}


void
MSNet::writeOutput() {
	// update detector values
	myDetectorControl->updateDetectors(myStep);
	const OptionsCont& oc = OptionsCont::getOptions();

	// check state dumps
	if (oc.isSet("netstate-dump")) {
		MSXMLRawOut::write(OutputDevice::getDeviceByOption("netstate-dump"), *myEdges, myStep,
			oc.getInt("netstate-dump.precision"));
	}

	// check fcd dumps
	if (OptionsCont::getOptions().isSet("fcd-output")) {
		MSFCDExport::write(OutputDevice::getDeviceByOption("fcd-output"), myStep, myHasElevation);
	}

	// check emission dumps
	if (OptionsCont::getOptions().isSet("emission-output")) {
		MSEmissionExport::write(OutputDevice::getDeviceByOption("emission-output"), myStep,
			oc.getInt("emission-output.precision"));
	}

	// battery dumps
	if (OptionsCont::getOptions().isSet("battery-output")) {
		MSBatteryExport::write(OutputDevice::getDeviceByOption("battery-output"), myStep,
			oc.getInt("battery-output.precision"));
	}

	// check full dumps
	if (OptionsCont::getOptions().isSet("full-output")) {
		MSFullExport::write(OutputDevice::getDeviceByOption("full-output"), myStep);
	}

	// check queue dumps
	if (OptionsCont::getOptions().isSet("queue-output")) {
		MSQueueExport::write(OutputDevice::getDeviceByOption("queue-output"), myStep);
	}

	// check amitran dumps
	if (OptionsCont::getOptions().isSet("amitran-output")) {
		MSAmitranTrajectories::write(OutputDevice::getDeviceByOption("amitran-output"), myStep);
	}

	// check vtk dumps
	if (OptionsCont::getOptions().isSet("vtk-output")) {

		if (MSNet::getInstance()->getVehicleControl().getRunningVehicleNo() > 0) {
			std::string timestep = time2string(myStep);
			timestep = timestep.substr(0, timestep.length() - 3);
			std::string output = OptionsCont::getOptions().getString("vtk-output");
			std::string filename = output + "_" + timestep + ".vtp";

			OutputDevice_File dev = OutputDevice_File(filename, false);

			//build a huge mass of xml files
			MSVTKExport::write(dev, myStep);

		}

	}

	// summary output
	if (OptionsCont::getOptions().isSet("summary-output")) {
		OutputDevice& od = OutputDevice::getDeviceByOption("summary-output");
		int departedVehiclesNumber = myVehicleControl->getDepartedVehicleNo();
		const double meanWaitingTime = departedVehiclesNumber != 0 ? myVehicleControl->getTotalDepartureDelay() / (double)departedVehiclesNumber : -1.;
		int endedVehicleNumber = myVehicleControl->getEndedVehicleNo();
		const double meanTravelTime = endedVehicleNumber != 0 ? myVehicleControl->getTotalTravelTime() / (double)endedVehicleNumber : -1.;
		od.openTag("step");
		od.writeAttr("time", time2string(myStep));
		od.writeAttr("loaded", myVehicleControl->getLoadedVehicleNo());
		od.writeAttr("inserted", myVehicleControl->getDepartedVehicleNo());
		od.writeAttr("running", myVehicleControl->getRunningVehicleNo());
		od.writeAttr("waiting", myInserter->getWaitingVehicleNo());
		od.writeAttr("ended", myVehicleControl->getEndedVehicleNo());
		od.writeAttr("meanWaitingTime", meanWaitingTime);
		od.writeAttr("meanTravelTime", meanTravelTime);
		od.writeAttr("halting", getHaltingVehicleNumber());
		std::pair<double, double> meanSpeed = getVehicleMeanSpeeds();
		od.writeAttr("meanSpeed", meanSpeed.first);
		od.writeAttr("meanSpeedRelative", meanSpeed.second);
		if (myLogExecutionTime) {
			od.writeAttr("duration", mySimStepDuration);
		}
		od.closeTag();
	}

	// write detector values
	myDetectorControl->writeOutput(myStep + DELTA_T, false);

	// write link states
	if (OptionsCont::getOptions().isSet("link-output")) {
		OutputDevice& od = OutputDevice::getDeviceByOption("link-output");
		od.openTag("timestep");
		od.writeAttr(SUMO_ATTR_ID, STEPS2TIME(myStep));
		const MSEdgeVector& edges = myEdges->getEdges();
		for (MSEdgeVector::const_iterator i = edges.begin(); i != edges.end(); ++i) {
			const std::vector<MSLane*>& lanes = (*i)->getLanes();
			for (std::vector<MSLane*>::const_iterator j = lanes.begin(); j != lanes.end(); ++j) {
				const std::vector<MSLink*>& links = (*j)->getLinkCont();
				for (std::vector<MSLink*>::const_iterator k = links.begin(); k != links.end(); ++k) {
					(*k)->writeApproaching(od, (*j)->getID());
				}
			}
		}
		od.closeTag();
	}

	// write SSM output
	for (std::set<MSDevice*>::iterator di = MSDevice_SSM::getInstances().begin(); di != MSDevice_SSM::getInstances().end(); ++di) {
		MSDevice_SSM* dev = static_cast<MSDevice_SSM*>(*di);
		dev->updateAndWriteOutput();
	}
}


bool
MSNet::logSimulationDuration() const {
	return myLogExecutionTime;
}


MSTransportableControl&
MSNet::getPersonControl() {
	if (myPersonControl == 0) {
		myPersonControl = new MSTransportableControl();
	}
	return *myPersonControl;
}

MSTransportableControl&
MSNet::getContainerControl() {
	if (myContainerControl == 0) {
		myContainerControl = new MSTransportableControl();
	}
	return *myContainerControl;
}


MSEdgeWeightsStorage&
MSNet::getWeightsStorage() {
	if (myEdgeWeights == 0) {
		myEdgeWeights = new MSEdgeWeightsStorage();
	}
	return *myEdgeWeights;
}


void
MSNet::preSimStepOutput() const {
	std::cout << "Step #" << time2string(myStep);
}


void
MSNet::postSimStepOutput() const {
	if (myLogExecutionTime) {
		std::ostringstream oss;
		oss.setf(std::ios::fixed, std::ios::floatfield);    // use decimal format
		oss.setf(std::ios::showpoint);    // print decimal point
		oss << std::setprecision(gPrecision);
		if (mySimStepDuration != 0) {
			const double durationSec = (double)mySimStepDuration / 1000.;
			oss << " (" << mySimStepDuration << "ms ~= "
				<< (TS / durationSec) << "*RT, ~"
				<< ((double)myVehicleControl->getRunningVehicleNo() / durationSec);
		}
		else {
			oss << " (0ms ?*RT. ?";
		}
		oss << "UPS, ";
#ifndef NO_TRACI
		if (TraCIServer::getInstance() != 0) {
			oss << "TraCI: " << myTraCIStepDuration << "ms, ";
		}
#endif
		oss << "vehicles TOT " << myVehicleControl->getDepartedVehicleNo()
			<< " ACT " << myVehicleControl->getRunningVehicleNo()
			<< " BUF " << myInserter->getWaitingVehicleNo()
			<< ")                                              ";
		std::string prev = "Step #" + time2string(myStep - DELTA_T);
		std::cout << oss.str().substr(0, 78 - prev.length());
	}
	std::cout << '\r';
}


void
MSNet::addVehicleStateListener(VehicleStateListener* listener) {
	if (find(myVehicleStateListeners.begin(), myVehicleStateListeners.end(), listener) == myVehicleStateListeners.end()) {
		myVehicleStateListeners.push_back(listener);
	}
}


void
MSNet::removeVehicleStateListener(VehicleStateListener* listener) {
	std::vector<VehicleStateListener*>::iterator i = find(myVehicleStateListeners.begin(), myVehicleStateListeners.end(), listener);
	if (i != myVehicleStateListeners.end()) {
		myVehicleStateListeners.erase(i);
	}
}


void
MSNet::informVehicleStateListener(const SUMOVehicle* const vehicle, VehicleState to) {
	for (std::vector<VehicleStateListener*>::iterator i = myVehicleStateListeners.begin(); i != myVehicleStateListeners.end(); ++i) {
		(*i)->vehicleStateChanged(vehicle, to);
	}
}


bool
MSNet::addStoppingPlace(const SumoXMLTag category, MSStoppingPlace* stop) {
	return myStoppingPlaces[category == SUMO_TAG_TRAIN_STOP ? SUMO_TAG_BUS_STOP : category].add(stop->getID(), stop);
}


MSStoppingPlace*
MSNet::getStoppingPlace(const std::string& id, const SumoXMLTag category) const {
	if (myStoppingPlaces.count(category) > 0) {
		return myStoppingPlaces.find(category)->second.get(id);
	}
	return 0;
}


std::string
MSNet::getStoppingPlaceID(const MSLane* lane, const double pos, const SumoXMLTag category) const {
	if (myStoppingPlaces.count(category) > 0) {
		for (const auto& it : myStoppingPlaces.find(category)->second) {
			MSStoppingPlace* stop = it.second;
			if (&stop->getLane() == lane && stop->getBeginLanePosition() <= pos && stop->getEndLanePosition() >= pos) {
				return stop->getID();
			}
		}
	}
	return "";
}


void
MSNet::writeChargingStationOutput() const {
	if (myStoppingPlaces.count(SUMO_TAG_CHARGING_STATION) > 0) {
		OutputDevice& output = OutputDevice::getDeviceByOption("chargingstations-output");
		for (const auto& it : myStoppingPlaces.find(SUMO_TAG_CHARGING_STATION)->second) {
			static_cast<MSChargingStation*>(it.second)->writeChargingStationOutput(output);
		}
	}
}


SUMOAbstractRouter<MSEdge, SUMOVehicle>&
MSNet::getRouterTT(const MSEdgeVector& prohibited) const {
	if (myRouterTT == 0) {
		const std::string routingAlgorithm = OptionsCont::getOptions().getString("routing-algorithm");
		if (routingAlgorithm == "dijkstra") {
			myRouterTT = new DijkstraRouter<MSEdge, SUMOVehicle, prohibited_withPermissions<MSEdge, SUMOVehicle> >(
				MSEdge::getAllEdges(), true, &MSNet::getTravelTime);
		}
		else {
			if (routingAlgorithm != "astar") {
				WRITE_WARNING("TraCI and Triggers cannot use routing algorithm '" + routingAlgorithm + "'. using 'astar' instead.");
			}
			myRouterTT = new AStarRouter<MSEdge, SUMOVehicle, prohibited_withPermissions<MSEdge, SUMOVehicle> >(
				MSEdge::getAllEdges(), true, &MSNet::getTravelTime);
		}
	}
	dynamic_cast<prohibited_withPermissions<MSEdge, SUMOVehicle>*>(myRouterTT)->prohibit(prohibited);
	return *myRouterTT;
}


SUMOAbstractRouter<MSEdge, SUMOVehicle>&
MSNet::getRouterEffort(const MSEdgeVector& prohibited) const {
	if (myRouterEffort == 0) {
		myRouterEffort = new DijkstraRouter<MSEdge, SUMOVehicle, prohibited_withPermissions<MSEdge, SUMOVehicle> >(
			MSEdge::getAllEdges(), true, &MSNet::getEffort, &MSNet::getTravelTime);
	}
	dynamic_cast<prohibited_withPermissions<MSEdge, SUMOVehicle>*>(myRouterEffort)->prohibit(prohibited);
	return *myRouterEffort;
}


MSNet::MSPedestrianRouterDijkstra&
MSNet::getPedestrianRouter(const MSEdgeVector& prohibited) const {
	if (myPedestrianRouter == 0) {
		myPedestrianRouter = new MSPedestrianRouterDijkstra();
	}
	myPedestrianRouter->prohibit(prohibited);
	return *myPedestrianRouter;
}


MSNet::MSIntermodalRouter&
MSNet::getIntermodalRouter(const MSEdgeVector& prohibited) const {
	if (myIntermodalRouter == 0) {
		int carWalk = 0;
		for (const std::string& opt : OptionsCont::getOptions().getStringVector("persontrip.transfer.car-walk")) {
			if (opt == "parkingAreas") {
				carWalk |= MSIntermodalRouter::PARKING_AREAS;
			}
			else if (opt == "ptStops") {
				carWalk |= MSIntermodalRouter::PT_STOPS;
			}
			else if (opt == "allJunctions") {
				carWalk |= MSIntermodalRouter::ALL_JUNCTIONS;
			}
		}
		myIntermodalRouter = new MSIntermodalRouter(MSNet::adaptIntermodalRouter, carWalk);
	}
	myIntermodalRouter->prohibit(prohibited);
	return *myIntermodalRouter;
}


void
MSNet::adaptIntermodalRouter(MSIntermodalRouter& router) {
	// add access to all parking areas
	for (const auto& i : myInstance->myStoppingPlaces[SUMO_TAG_PARKING_AREA]) {
		router.addAccess(i.first, &i.second->getLane().getEdge(), i.second->getAccessPos(&i.second->getLane().getEdge()), SUMO_TAG_PARKING_AREA);
	}
	// add access to all public transport stops
	for (const auto& i : myInstance->myStoppingPlaces[SUMO_TAG_BUS_STOP]) {
		router.addAccess(i.first, &i.second->getLane().getEdge(), i.second->getAccessPos(&i.second->getLane().getEdge()), SUMO_TAG_BUS_STOP);
		for (const auto& a : i.second->getAllAccessPos()) {
			router.addAccess(i.first, &a.first->getEdge(), a.second, SUMO_TAG_BUS_STOP);
		}
	}
	myInstance->getInsertionControl().adaptIntermodalRouter(router);
	myInstance->getVehicleControl().adaptIntermodalRouter(router);
}


const NamedRTree&
MSNet::getLanesRTree() const {
	if (!myLanesRTree.first) {
		MSLane::fill(myLanesRTree.second);
		myLanesRTree.first = true;
	}
	return myLanesRTree.second;
}


bool
MSNet::checkElevation() {
	const MSEdgeVector& edges = myEdges->getEdges();
	for (MSEdgeVector::const_iterator e = edges.begin(); e != edges.end(); ++e) {
		for (std::vector<MSLane*>::const_iterator i = (*e)->getLanes().begin(); i != (*e)->getLanes().end(); ++i) {
			if ((*i)->getShape().hasElevation()) {
				return true;
			}
		}
	}
	return false;
}


int
MSNet::getHaltingVehicleNumber() const {
	int result = 0;
	for (MSVehicleControl::constVehIt it = myVehicleControl->loadedVehBegin(); it != myVehicleControl->loadedVehEnd(); ++it) {
		const SUMOVehicle* veh = it->second;
		if ((veh->isOnRoad() || veh->isRemoteControlled()) && veh->getSpeed() < SUMO_const_haltingSpeed) {
			result++;
		}
	}
	return result;
}


std::pair<double, double>
MSNet::getVehicleMeanSpeeds() const {
	double speedSum = 0;
	double relSpeedSum = 0;
	int count = 0;
	for (MSVehicleControl::constVehIt it = myVehicleControl->loadedVehBegin(); it != myVehicleControl->loadedVehEnd(); ++it) {
		const SUMOVehicle* veh = it->second;
		if ((veh->isOnRoad() || veh->isRemoteControlled()) && !veh->isStopped()) {
			count++;
			speedSum += veh->getSpeed();
			relSpeedSum += veh->getSpeed() / veh->getEdge()->getSpeedLimit();
		}
	}
	if (count > 0) {
		return std::make_pair(speedSum / count, relSpeedSum / count);
	}
	else {
		return std::make_pair(-1, -1);
	}
}


/****************************************************************************/
