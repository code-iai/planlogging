#include <planlogging/CPlanLoggerROS.h>


CPlanLoggerROS::CPlanLoggerROS() {
  this->startServices();
}

CPlanLoggerROS::~CPlanLoggerROS() {
  this->stopServices();
}

void CPlanLoggerROS::startServices() {
  ros::NodeHandle nhPrivate("~");
  
  m_srvStartNode = nhPrivate.advertiseService<CPlanLoggerROS>("start_node", &CPlanLoggerROS::serviceCallbackStartNode, this);
  m_srvStopNode = nhPrivate.advertiseService<CPlanLoggerROS>("stop_node", &CPlanLoggerROS::serviceCallbackStopNode, this);
  m_srvAlterNode = nhPrivate.advertiseService<CPlanLoggerROS>("alter_node", &CPlanLoggerROS::serviceCallbackAlterNode, this);
  m_srvControl = nhPrivate.advertiseService<CPlanLoggerROS>("control", &CPlanLoggerROS::serviceCallbackControl, this);
}

void CPlanLoggerROS::stopServices() {
  m_srvStartNode.shutdown();
  m_srvStopNode.shutdown();
  m_srvAlterNode.shutdown();
  m_srvControl.shutdown();
}

bool CPlanLoggerROS::serviceCallbackStartNode(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
  bool bReturnvalue = false;
  CDesignator *desigRequest = new CDesignator(req.request.designator);
  int nDetailLevel = (int)desigRequest->floatValue("_detail-level");
  
  if(desigRequest) {
    string strDesigType = "UNKNOWN";
    switch(desigRequest->type()) {
    case ACTION:
      strDesigType = "ACTION";
      break;
      
    case OBJECT:
      strDesigType = "OBJECT";
      break;
      
    case LOCATION:
      strDesigType = "LOCATION";
      break;
      
    default:
      break;
    }
    
    ROS_INFO("Received start node designator of type %s (detail-level: %d).",
	     strDesigType.c_str(), nDetailLevel);
    CPlanNode *pnStart = this->addPlanNode(desigRequest->stringValue("_name"));
    pnStart->setDetailLevel(nDetailLevel);
    pnStart->setDescription(desigRequest->children());
    pnStart->setSource(desigRequest->stringValue("_source"));
    
    CDesignator *desigResponse = new CDesignator();
    desigResponse->setType(ACTION);
    desigResponse->setValue(string("_id"), pnStart->id());
    
    res.response.designators.push_back(desigResponse->serializeToMessage());
    
    delete desigResponse;
    delete desigRequest;
    
    bReturnvalue = true;
  }
  
  return bReturnvalue;
}

bool CPlanLoggerROS::serviceCallbackStopNode(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
  bool bReturnvalue = false;
  CDesignator *desigRequest = new CDesignator(req.request.designator);
  
  if(desigRequest) {
    int nID = (int)desigRequest->floatValue("_id");
    int nSuccess = (int)desigRequest->floatValue("_success");
    CPlanNode *pnCurrent = this->activeNode();
    
    if(pnCurrent) {
      if(pnCurrent->id() == nID) {
	ROS_INFO("Received stop node designator for ID %d (success: %s).", nID, (nSuccess ? "yes" : "no"));
	
	pnCurrent->setSuccess((nSuccess == 1 ? true : false));
	
	CPlanNode *pnParent = pnCurrent->parent();
	this->setNodeAsActive(pnParent);
	
	bReturnvalue = true;
      } else {
	ROS_WARN("Received stop node designator for ID %d while ID %d is active.", nID, pnCurrent->id());
      }
    } else {
      ROS_WARN("Received stop node designator for ID %d while in top-level.", nID);
    }
    
    delete desigRequest;
  }
  
  return bReturnvalue;
}

bool CPlanLoggerROS::serviceCallbackAlterNode(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
  return false;
}

bool CPlanLoggerROS::serviceCallbackControl(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
  bool bReturnvalue = false;
  CDesignator *desigRequest = new CDesignator(req.request.designator);
  
  if(desigRequest) {
    string strCommand = desigRequest->stringValue("command");
    
    if(strCommand == "EXTRACT") {
      string strFormat = desigRequest->stringValue("format");
      
      if(strFormat == "DOT") {
	string strFilename = desigRequest->stringValue("filename");
	
	if(strFilename != "") {
	  int nMaxDetailLevel = (int)desigRequest->floatValue("max-detail-level");
	  int nSuccesses = (int)desigRequest->floatValue("show-successes");
	  int nFails = (int)desigRequest->floatValue("show-fails");
	  bool bSuccesses = (nSuccesses == 1 ? true : false);
	  bool bFails = (nFails == 1 ? true : false);
	  
	  string strContents = this->generateDotDiGraph(bSuccesses, bFails, nMaxDetailLevel);
	  
	  ofstream myfile;
	  myfile.open(strFilename.c_str());
	  myfile << strContents;
	  myfile.close();
	  
	  ROS_INFO("Extracted plan nodes to .dot digraph in file '%s'.", strFilename.c_str());
	  ROS_INFO("Options:");
	  if(bSuccesses) {
	    ROS_INFO(" - show successes = yes");
	  }
	  
	  if(bFails) {
	    ROS_INFO(" - show fails = yes");
	  }
	  
	  ROS_INFO(" - max detail level = %d", nMaxDetailLevel);
	  
	  bReturnvalue = true;
	}
      } else {
	ROS_WARN("Unknown output format: '%s'", strFormat.c_str());
      }
    } else if(strCommand == "USE-COLOR") {
      int nUseColor = (int)desigRequest->floatValue("use-color");
      bool bUseColor = (nUseColor == 1 ? true : false);
      
      this->setUseColor(bUseColor);
      
      ROS_INFO("Now using colors: %s", (bUseColor ? "yes" : "no"));
      
      bReturnvalue = true;
    } else {
      ROS_WARN("Unknown control command: '%s'", strCommand.c_str());
    }
  }
  
  return bReturnvalue;
}
