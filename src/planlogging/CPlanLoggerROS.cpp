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
    pnStart->setStartTime(this->getTimeStamp());
    
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
	pnCurrent->setEndTime(this->getTimeStamp());
	
	CPlanNode *pnParent = pnCurrent->parent();
	this->setNodeAsActive(pnParent);
	
	while(pnParent) {
	  if(pnParent->prematurelyEnded()) {
	    pnParent = pnParent->parent();
	    this->setNodeAsActive(pnParent);
	  } else {
	    this->setNodeAsActive(pnParent);
	    break;
	  }
	}
	
	bReturnvalue = true;
      } else {
	ROS_WARN("Received stop node designator for ID %d while ID %d is active.", nID, pnCurrent->id());
	
	CPlanNode *pnEndedPrematurely = NULL;
	CPlanNode *pnSearchTemp = pnCurrent->parent();
	
	while(pnSearchTemp) {
	  if(pnSearchTemp->id() == nID) {
	    pnEndedPrematurely = pnSearchTemp;
	    pnSearchTemp = NULL;
	  } else {
	    pnSearchTemp = pnSearchTemp->parent();
	  }
	}
	
	if(pnEndedPrematurely) {
	  // Found the prematurely ended node in this branch
	  ROS_INFO("Marking node %d as prematurely ended.", nID);
	  pnEndedPrematurely->setPrematurelyEnded(true);
	  bReturnvalue = true;
	} else {
	  // Didn't find the prematurely ended node in this branch
	  ROS_ERROR("The apparently prematurely ended node %d was not found. This is probably a problem.", nID);
	}
      }
    } else {
      ROS_WARN("Received stop node designator for ID %d while in top-level.", nID);
    }
    
    delete desigRequest;
  }
  
  return bReturnvalue;
}

bool CPlanLoggerROS::serviceCallbackAlterNode(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
bool bReturnvalue = false;
  CDesignator *desigRequest = new CDesignator(req.request.designator);
  
  if(desigRequest) {
    string strCommand = desigRequest->stringValue("command");
    
    if(strCommand == "ADD-IMAGE") {
      string strImageOrigin = desigRequest->stringValue("origin");
      string strImageFilename = desigRequest->stringValue("filename");
      
      if(this->activeNode()) {
	CImageCapturer *capImage = new CImageCapturer();
	
	if(capImage->captureFromTopic(strImageOrigin, strImageFilename)) {
	  this->activeNode()->addImage(strImageOrigin, strImageFilename);
	  ROS_INFO("Added image '%s' (from '%s') to active node (id %d).", strImageFilename.c_str(), strImageOrigin.c_str(), this->activeNode()->id());
	  
	  bReturnvalue = true;
	} else {
	  ROS_WARN("Unable to capture image from topic `%s'.", strImageOrigin.c_str());
	}
	
	delete capImage;
      } else {
	ROS_WARN("No node context available. Cannot add image while on top-level.");
      }
    } else if(strCommand == "ADD-OBJECT") {
      CKeyValuePair *ckvpDesc = desigRequest->childForKey("DESCRIPTION");
      
      if(ckvpDesc) {
	list<CKeyValuePair*> lstDesc = ckvpDesc->children();
	
	if(this->activeNode()) {
	  this->activeNode()->addObject(lstDesc);
	  ROS_INFO("Added object to active node (id %d).", this->activeNode()->id());
	  
	  bReturnvalue = true;
	} else {
	  ROS_WARN("No node context available. Cannot add object while on top-level.");
	}
      }
    } else {
      ROS_WARN("Unknown alter command: '%s'", strCommand.c_str());
    }
  }
  
  return bReturnvalue;
}

bool CPlanLoggerROS::serviceCallbackControl(designator_integration_msgs::DesignatorCommunication::Request &req, designator_integration_msgs::DesignatorCommunication::Response &res) {
  bool bReturnvalue = false;
  CDesignator *desigRequest = new CDesignator(req.request.designator);
  
  if(desigRequest) {
    string strCommand = desigRequest->stringValue("command");
    
    if(strCommand == "SESSION") {
      string strSessionCommand = desigRequest->stringValue("session-command");
      
      if(strSessionCommand == "RESTART") {
	ROS_INFO("Request to renew session. Flushing buffers and resetting experiment name.");
	
	if(this->renewSession()) {
	  ROS_INFO("New experiment name is: '%s'", this->experimentName().c_str());
	  bReturnvalue = true;
	} else {
	  ROS_ERROR("Could not renew session.");
	}
      } else {
	ROS_WARN("Unknown session command: '%s'", strCommand.c_str());
      }
    } else if(strCommand == "EXTRACT") {
      string strFormat = desigRequest->stringValue("format");
      
      if(strFormat == "DOT") {
	string strFilename = desigRequest->stringValue("filename");
	
	if(strFilename != "") {
	  CExporterDot* expDot = new CExporterDot();
	  this->configureExporter(expDot);
	  expDot->configuration()->setValue(string("display-successes"), (int)desigRequest->floatValue("show-successes"));
	  expDot->configuration()->setValue(string("display-failures"), (int)desigRequest->floatValue("show-fails"));
	  expDot->configuration()->setValue(string("max-detail-level"), (int)desigRequest->floatValue("max-detail-level"));
	  
	  expDot->setOutputFilename(this->experimentsResultRoot() + "/" + this->experimentName() + "/" + strFilename);
	  
	  if(expDot->runExporter(NULL)) {
	    bReturnvalue = true;
	  }
	  
	  int nMaxDetailLevel = (int)desigRequest->floatValue("max-detail-level");
	  int nSuccesses = (int)desigRequest->floatValue("show-successes");
	  int nFails = (int)desigRequest->floatValue("show-fails");
	  bool bSuccesses = (nSuccesses == 1 ? true : false);
	  bool bFails = (nFails == 1 ? true : false);
	  
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
      } else if(strFormat == "OWL") {
	string strFilename = desigRequest->stringValue("filename");
	
	if(strFilename != "") {
	  int nMaxDetailLevel = (int)desigRequest->floatValue("max-detail-level");
	  int nSuccesses = (int)desigRequest->floatValue("show-successes");
	  int nFails = (int)desigRequest->floatValue("show-fails");
	  bool bSuccesses = (nSuccesses == 1 ? true : false);
	  bool bFails = (nFails == 1 ? true : false);
	  
	  string strContents = this->generateOWL(bSuccesses, bFails, nMaxDetailLevel);
	  
	  string strFullFilename = this->experimentPath() + strFilename;
	  ofstream myfile;
	  myfile.open(strFullFilename.c_str());
	  myfile << strContents;
	  myfile.close();
	  
	  ROS_INFO("Extracted plan nodes to .owl in file '%s'.", strFilename.c_str());
	  ROS_INFO("Options:");
	  if(bSuccesses) {
	    ROS_INFO(" - show successes = yes");
	  }
	  
	  if(bFails) {
	    ROS_INFO(" - show fails = yes");
	  }
	  
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
