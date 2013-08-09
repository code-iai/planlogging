#include <planlogging/CPlanLogger.h>


CPlanLogger::CPlanLogger() {
  m_pnActive = NULL;
  
  srand(time(NULL));
}

CPlanLogger::~CPlanLogger() {
  this->clearPlanNodes();
}

CPlanNode* CPlanLogger::addPlanNode(string strName) {
  int nID = this->nextFreeIndex();
  CPlanNode *pnNew = new CPlanNode(nID, strName);
  
  if(m_pnActive == NULL) {
    // Add a new top-level node
    m_lstPlanNodes.push_back(pnNew);
    
    cout << "Adding new top-level plan node with ID " << nID << endl;
  } else {
    // Add it as a subnode to the current contextual node
    m_pnActive->addSubnode(pnNew);
    
    cout << "Adding new plan subnode with ID " << nID << endl;
  }
  
  m_lstNodeList.push_back(pnNew);
  
  cout << "The new node's name is '" << strName << "'" << endl;
  this->setNodeAsActive(pnNew);
  
  return pnNew;
}

void CPlanLogger::setNodeAsActive(CPlanNode *pnActive) {
  m_pnActive = pnActive;
  
  if(m_pnActive) {
    cout << "Setting node ID " << m_pnActive->id() << " as active node" << endl;
  } else {
    cout << "Removed active node, returning to top-level" << endl;
  }
}

int CPlanLogger::nextFreeIndex() {
  int nHighestID = -1;
  
  for(list<CPlanNode*>::iterator itNode = m_lstPlanNodes.begin();
      itNode != m_lstPlanNodes.end();
      itNode++) {
    CPlanNode *pnCurrent = *itNode;
    
    nHighestID = max(nHighestID, pnCurrent->highestID());
  }
  
  return nHighestID + 1;
}

CPlanNode* CPlanLogger::planNodeForID(int nID) {
  CPlanNode* pnFind = NULL;
  
  for(list<CPlanNode*>::iterator itNode = m_lstPlanNodes.begin();
      itNode != m_lstPlanNodes.end();
      itNode++) {
    CPlanNode *pnCurrent = *itNode;
    
    if(pnCurrent->id() == nID) {
      pnFind = pnCurrent;
      break;
    }
  }
  
  return pnFind;
}

void CPlanLogger::clearPlanNodes() {
  for(list<CPlanNode*>::iterator itNode = m_lstPlanNodes.begin();
      itNode != m_lstPlanNodes.end();
      itNode++) {
    CPlanNode *pnCurrent = *itNode;
    delete pnCurrent;
  }
  
  m_lstPlanNodes.clear();
  m_lstNodeList.clear();
  
  cout << "Cleared all nodes from the plan log" << endl;
}

CPlanNode* CPlanLogger::activeNode() {
  return m_pnActive;
}

CNode* CPlanLogger::convertPlanNodeToNode(CPlanNode* pnConvert) {
  CNode *ndNew = new CNode(pnConvert->description());
  
  // Meta Information
  stringstream stsTimeStart;
  stsTimeStart << pnConvert->startTime();
  stringstream stsTimeEnd;
  stsTimeEnd << pnConvert->endTime();
  
  ndNew->metaInformation()->setValue(string("time-start"), stsTimeStart.str());
  ndNew->metaInformation()->setValue(string("time-end"), stsTimeEnd.str());
  ndNew->metaInformation()->setValue(string("success"), (pnConvert->success() ? 1 : 0));
  ndNew->metaInformation()->setValue(string("source"), pnConvert->source());
  ndNew->metaInformation()->setValue(string("prematurely-ended"), (pnConvert->prematurelyEnded() ? 1 : 0));
  ndNew->metaInformation()->setValue(string("detail-level"), pnConvert->detailLevel());
  ndNew->setTitle(pnConvert->name());
  
  // Attached Images
  CKeyValuePair* ckvpImages = ndNew->metaInformation()->addChild("images");
  list<CImage*> lstImages = pnConvert->images();
  unsigned int unIndex = 0;
  for(list<CImage*>::iterator itImage = lstImages.begin();
      itImage != lstImages.end();
      itImage++) {
    CImage *imgCurrent = *itImage;
    
    stringstream sts;
    sts << "image-";
    sts << unIndex;
    
    CKeyValuePair* ckvpImage = ckvpImages->addChild(sts.str());
    ckvpImage->setValue(string("origin"), imgCurrent->origin());
    ckvpImage->setValue(string("filename"), imgCurrent->filename());
    
    unIndex++;
  }
  
  // Attached Objects
  CKeyValuePair* ckvpObjects = ndNew->metaInformation()->addChild("objects");
  list<CObject*> lstObjects = pnConvert->objects();
  unIndex = 0;
  for(list<CObject*>::iterator itObject = lstObjects.begin();
      itObject != lstObjects.end();
      itObject++) {
    CObject *objCurrent = *itObject;
    
    stringstream sts;
    sts << "object-";
    sts << unIndex;
    
    CKeyValuePair* ckvpObject = ckvpObjects->addChild(sts.str());
    list<CKeyValuePair*> lstDescription = objCurrent->description();
    for(list<CKeyValuePair*>::iterator itPair = lstDescription.begin();
  	itPair != lstDescription.end();
  	itPair++) {
      ckvpObject->addChild(*itPair);
    }
    
    unIndex++;
  }
  
  // Subnodes
  list<CPlanNode*> lstChildren = pnConvert->subnodes();
  for(list<CPlanNode*>::iterator itChild = lstChildren.begin();
      itChild != lstChildren.end();
      itChild++) {
    CPlanNode *pnCurrent = *itChild;
    
    ndNew->addSubnode(this->convertPlanNodeToNode(pnCurrent));
  }
  
  return ndNew;
}

void CPlanLogger::configureExporter(CExporter *expConfigure) {
  for(list<CPlanNode*>::iterator itNode = m_lstPlanNodes.begin();
      itNode != m_lstPlanNodes.end();
      itNode++) {
    CPlanNode *pnCurrent = *itNode;
    
    expConfigure->addNode(this->convertPlanNodeToNode(pnCurrent));
  }
}

string CPlanLogger::generateRandomIdentifier(string strPrefix, unsigned int unLength) {
  stringstream sts;
  sts << strPrefix;
  
  for(unsigned int unI = 0; unI < unLength; unI++) {
    int nRandom;
    do {
      nRandom = rand() % 122 + 48;
    } while(nRandom < 48 ||
	    (nRandom > 57 && nRandom < 65) ||
	    (nRandom > 90 && nRandom < 97) ||
	    nRandom > 122);
    
    char cRandom = (char)nRandom;
    sts << cRandom;
  }
  
  return sts.str();
}

string CPlanLogger::replaceString(string strOriginal, string strReplaceWhat, string strReplaceBy) {
  size_t found;
  
  found = strOriginal.find(strReplaceWhat);
  while(found != string::npos) {
    strOriginal.replace(found, strReplaceWhat.length(), strReplaceBy);
    found = strOriginal.find(strReplaceWhat, found + strReplaceBy.length());
  };
  
  return strOriginal;
}

int CPlanLogger::getTimeStamp() {
  return std::time(0);
}

void CPlanLogger::fillPlanNodesUniqueIDs() {
  // Plan nodes
  for(list<CPlanNode*>::iterator itNode = m_lstNodeList.begin();
      itNode != m_lstNodeList.end();
      itNode++) {
    (*itNode)->setUniqueID(this->generateRandomIdentifier("event_", 8));
    
    // Objects attached to plan nodes
    list<CObject*> lstObjects = (*itNode)->objects();
    for(list<CObject*>::iterator itObject = lstObjects.begin();
	itObject != lstObjects.end();
	itObject++) {
      (*itObject)->setUniqueID(this->generateRandomIdentifier("object_", 8));
    }
  }
}

void CPlanLogger::setExperimentsResultRoot(string strExperimentsResultRoot) {
  m_strExperimentsResultRoot = strExperimentsResultRoot;
  
  mkdir(this->experimentsResultRoot().c_str(), 0777);
}

string CPlanLogger::experimentsResultRoot() {
  return m_strExperimentsResultRoot;
}

void CPlanLogger::setExperimentName(string strExperimentName) {
  m_strExperimentName = strExperimentName;
}

string CPlanLogger::experimentName() {
  return m_strExperimentName;
}

bool CPlanLogger::renewSession() {
  bool bReturnvalue = false;
  
  if(m_strExperimentsResultRoot != "") {
    // Get rid of old plan node data
    this->clearPlanNodes();
    m_pnActive = NULL;
    
    // Generate new experiment name (plus according directory)
    int nIndex = 0;
    bool bExists;
    string strNewName;
    
    do {
      stringstream sts;
      sts << "experiment-";
      sts << nIndex;
      nIndex++;
      
      strNewName = sts.str();
      string strNewPath = this->experimentPath(strNewName);
      
      struct stat sb;
      int nReturnStat = stat(strNewPath.c_str(), &sb);
      bExists = (nReturnStat == 0);
    } while(bExists);
    
    m_strExperimentName = strNewName;
    mkdir(this->experimentPath().c_str(), 0777);
    ROS_INFO("Name of new experiment is '%s', setting symlink accordingly.", m_strExperimentName.c_str());
    
    string strExpDir = this->experimentsResultRoot() + "/" + this->experimentName();
    string strSymlinkDir = this->experimentsResultRoot() + "/current-experiment";
    
    remove(strSymlinkDir.c_str());
    symlink(strExpDir.c_str(), strSymlinkDir.c_str());
    
    bReturnvalue = true;
  } else {
    ROS_WARN("Cannot renew experiment name. Experiments result root is empty.");
    ROS_WARN("Did you move the package in some other folder than 'planlogger'?");
  }
  
  return bReturnvalue;
}

string CPlanLogger::experimentPath(string strExperimentName) {
  if(strExperimentName == "") {
    strExperimentName = m_strExperimentName;
  }
  
  return m_strExperimentsResultRoot + "/" + strExperimentName + "/";
}
