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
    CNode *ndStart = this->addNode(desigRequest->stringValue("_name"));
    ndStart->setDescription(desigRequest->children());
    ndStart->metaInformation()->setValue(string("source"), desigRequest->stringValue("_source"));
    
    stringstream stsTimeStart;
    stsTimeStart << this->getTimeStamp();
    ndStart->metaInformation()->setValue(string("time-start"), stsTimeStart.str());
    ndStart->metaInformation()->setValue(string("detail-level"), nDetailLevel);
    
    CDesignator *desigResponse = new CDesignator();
    desigResponse->setType(ACTION);
    desigResponse->setValue(string("_id"), ndStart->id());
    
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
    CNode *ndCurrent = this->activeNode();
    
    if(ndCurrent) {
      if(ndCurrent->id() == nID) {
	ROS_INFO("Received stop node designator for ID %d (success: %s).", nID, (nSuccess ? "yes" : "no"));
	
	ndCurrent->metaInformation()->setValue(string("success"), nSuccess);
	stringstream stsTimeEnd;
	stsTimeEnd << this->getTimeStamp();
	ndCurrent->metaInformation()->setValue(string("time-end"), stsTimeEnd.str());
	
	CNode *ndParent = ndCurrent->parent();
	this->setNodeAsActive(ndParent);
	
	while(ndParent) {
	  if(ndParent->prematurelyEnded()) {
	    ndParent = ndParent->parent();
	    this->setNodeAsActive(ndParent);
	  } else {
	    this->setNodeAsActive(ndParent);
	    break;
	  }
	}
	
	bReturnvalue = true;
      } else {
	ROS_WARN("Received stop node designator for ID %d while ID %d is active.", nID, ndCurrent->id());
	
	CNode *ndEndedPrematurely = NULL;
	CNode *ndSearchTemp = ndCurrent->parent();
	
	while(ndSearchTemp) {
	  if(ndSearchTemp->id() == nID) {
	    ndEndedPrematurely = ndSearchTemp;
	    ndSearchTemp = NULL;
	  } else {
	    ndSearchTemp = ndSearchTemp->parent();
	  }
	}
	
	if(ndEndedPrematurely) {
	  // Found the prematurely ended node in this branch
	  ROS_INFO("Marking node %d as prematurely ended.", nID);
	  ndEndedPrematurely->setPrematurelyEnded(true);
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
	
	if(capImage->captureFromTopic(strImageOrigin, strImageFilename, this->experimentsResultRoot() + "/" + this->experimentName() + "/")) {
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
    } else if(strCommand == "ADD-FAILURE") {
      // Adding a failure to a node also means to set its success state to 'false'.
      string strCondition = desigRequest->stringValue("condition");
      
      if(this->activeNode()) {
	stringstream stsTimeFail;
	stsTimeFail << this->getTimeStamp();
	
	this->activeNode()->addFailure(strCondition, stsTimeFail.str());
	this->activeNode()->setSuccess(false);
	ROS_INFO("Added failure to active node (id %d): '%s'", this->activeNode()->id(), strCondition.c_str());
	
	bReturnvalue = true;
      } else {
	ROS_WARN("No node context available. Cannot add object while on top-level.");
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
	  } else {
	    ROS_WARN("Failed to export .dot file.");
	  }
	}
      } else if(strFormat == "OWL") {
	string strFilename = desigRequest->stringValue("filename");
	
	if(strFilename != "") {
	  CExporterOwl* expOwl = new CExporterOwl();
	  this->configureExporter(expOwl);
	  expOwl->configuration()->setValue(string("display-successes"), (int)desigRequest->floatValue("show-successes"));
	  expOwl->configuration()->setValue(string("display-failures"), (int)desigRequest->floatValue("show-fails"));
	  expOwl->configuration()->setValue(string("max-detail-level"), (int)desigRequest->floatValue("max-detail-level"));
	  
	  expOwl->setOutputFilename(this->experimentsResultRoot() + "/" + this->experimentName() + "/" + strFilename);
	  
	  if(expOwl->runExporter(NULL)) {
	    int nMaxDetailLevel = (int)desigRequest->floatValue("max-detail-level");
	    int nSuccesses = (int)desigRequest->floatValue("show-successes");
	    int nFails = (int)desigRequest->floatValue("show-fails");
	    bool bSuccesses = (nSuccesses == 1 ? true : false);
	    bool bFails = (nFails == 1 ? true : false);
	    
	    ROS_INFO("Extracted plan nodes to .owl data in file '%s'.", strFilename.c_str());
	    ROS_INFO("Options:");
	    if(bSuccesses) {
	      ROS_INFO(" - show successes = yes");
	    }
	    
	    if(bFails) {
	      ROS_INFO(" - show fails = yes");
	    }
	    
	    ROS_INFO(" - max detail level = %d", nMaxDetailLevel);
	    
	    bReturnvalue = true;
	  } else {
	    ROS_WARN("Failed to export .owl file.");
	  }
	}
      } else if(strFormat == "MONGO") {
	string strDatabase = desigRequest->stringValue("database");
	string strCollection = desigRequest->stringValue("collection");
	
	CExporterMongoDB *expMongo = new CExporterMongoDB();
	expMongo->setDatabaseName(strDatabase);
	expMongo->setCollectionName(strCollection);
	expMongo->setExperimentName(this->experimentName());
	
	expMongo->configuration()->setValue(string("display-successes"), (int)desigRequest->floatValue("show-successes"));
	expMongo->configuration()->setValue(string("display-failures"), (int)desigRequest->floatValue("show-fails"));
	expMongo->configuration()->setValue(string("max-detail-level"), (int)desigRequest->floatValue("max-detail-level"));
	this->configureExporter(expMongo);
	
	if(expMongo->runExporter(NULL)) {
	  int nMaxDetailLevel = (int)desigRequest->floatValue("max-detail-level");
	  int nSuccesses = (int)desigRequest->floatValue("show-successes");
	  int nFails = (int)desigRequest->floatValue("show-fails");
	  bool bSuccesses = (nSuccesses == 1 ? true : false);
	  bool bFails = (nFails == 1 ? true : false);
	  
	  ROS_INFO("Extracted plan nodes to MongoDB '%s/%s'.", strDatabase.c_str(), strCollection.c_str());
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
	
	delete expMongo;
      } else {
	ROS_WARN("Unknown output format: '%s'", strFormat.c_str());
      }
    } else {
      ROS_WARN("Unknown control command: '%s'", strCommand.c_str());
    }
  }
  
  return bReturnvalue;
}
