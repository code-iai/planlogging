#include <planlogging/CPlanLogger.h>


/*! \brief Constructor for the base CPlanLogger class.
  
  The constructor sets the initial values of internal variables, and
  initialized the random number generator for generating unique node
  IDs.

*/
CPlanLogger::CPlanLogger() {
  m_pnActive = NULL;
  
  srand(time(NULL));
}

/*! \brief Destructor for the base CPlanLogger class.

  The destructor deletes all plan nodes currently stored in its memory.

*/
CPlanLogger::~CPlanLogger() {
  this->clearPlanNodes();
}

/*! \brief Adding a new plan node by name to the current hierarchy.

  This function creates a new instance of type CPlanNode, assigns it
  the given name strName, and adds this node to the current plan event
  hierarchy level.
  
  \param strName Name of the new node to be placed into the hierarchy.
  
  \return Pointer to the newly created instance of type CPlanNode.

*/
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

/*! \brief Sets the given plan node as currently active node in the hierarchy.
  
  New nodes added to the hierarchy are added as child nodes to this node.
  
  \param pnActive Pointer to the CPlanNode class instance used as the new hierarchy root for new nodes.

*/
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

/*! \brief Deletes all plan nodes.
  
  The current hierarchy is cleared and all plan nodes stored in it are deleted.

*/
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

/*! \brief Get the currently active node.
  
  The node that is currently set as hierarchical root for new nodes is returned.
  
  \return Pointer to the CPlanNode instance that is the current hierarchy root node.

*/
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

/*! \brief Configures the given exporter with all current plan nodes.
  
  The currently known hierarchy of plan nodes is transferred into the given exporter in order to be processed.
  
  \param expConfigure The exporter to configure with the current content of the plan event log.

*/
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
