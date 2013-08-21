#include <planlogging/CPlanLogger.h>


/*! \brief Constructor for the base CPlanLogger class.
  
  The constructor sets the initial values of internal variables, and
  initialized the random number generator for generating unique node
  IDs.

*/
CPlanLogger::CPlanLogger() {
  m_ndActive = NULL;
  
  srand(time(NULL));
}

/*! \brief Destructor for the base CPlanLogger class.

  The destructor deletes all plan nodes currently stored in its memory.

*/
CPlanLogger::~CPlanLogger() {
  this->clearNodes();
}

/*! \brief Adding a new plan node by name to the current hierarchy.

  This function creates a new instance of type CPlanNode, assigns it
  the given name strName, and adds this node to the current plan event
  hierarchy level.
  
  \param strName Name of the new node to be placed into the hierarchy.
  
  \return Pointer to the newly created instance of type CPlanNode.

*/
CNode* CPlanLogger::addNode(string strName) {
  int nID = this->nextFreeIndex();
  CNode *ndNew = new CNode(strName);
  ndNew->setID(nID);
  
  if(m_ndActive == NULL) {
    // Add a new top-level node
    m_lstNodes.push_back(ndNew);
    
    cout << "Adding new top-level node with ID " << nID << endl;
  } else {
    // Add it as a subnode to the current contextual node
    m_ndActive->addSubnode(ndNew);
    
    cout << "Adding new subnode with ID " << nID << endl;
  }
  
  cout << "The new node's name is '" << strName << "'" << endl;
  this->setNodeAsActive(ndNew);
  
  return ndNew;
}

/*! \brief Sets the given plan node as currently active node in the hierarchy.
  
  New nodes added to the hierarchy are added as child nodes to this node.
  
  \param pnActive Pointer to the CPlanNode class instance used as the new hierarchy root for new nodes.

*/
void CPlanLogger::setNodeAsActive(CNode *ndActive) {
  m_ndActive = ndActive;
  
  if(m_ndActive) {
    cout << "Setting node ID " << m_ndActive->id() << " as active node" << endl;
  } else {
    cout << "Removed active node, returning to top-level" << endl;
  }
}

int CPlanLogger::nextFreeIndex() {
  int nHighestID = -1;
  
  for(list<CNode*>::iterator itNode = m_lstNodes.begin();
      itNode != m_lstNodes.end();
      itNode++) {
    CNode *ndCurrent = *itNode;
    
    nHighestID = max(nHighestID, ndCurrent->highestID());
  }
  
  return nHighestID + 1;
}

CNode* CPlanLogger::nodeForID(int nID) {
  CNode* ndFind = NULL;
  
  for(list<CNode*>::iterator itNode = m_lstNodes.begin();
      itNode != m_lstNodes.end();
      itNode++) {
    CNode *ndCurrent = *itNode;
    
    if(ndCurrent->id() == nID) {
      ndFind = ndCurrent;
      break;
    }
  }
  
  return ndFind;
}

/*! \brief Deletes all plan nodes.
  
  The current hierarchy is cleared and all plan nodes stored in it are deleted.

*/
void CPlanLogger::clearNodes() {
  for(list<CNode*>::iterator itNode = m_lstNodes.begin();
      itNode != m_lstNodes.end();
      itNode++) {
    CNode *ndCurrent = *itNode;
    delete ndCurrent;
  }
  
  m_lstNodes.clear();
  
  cout << "Cleared all nodes from the plan log" << endl;
}

/*! \brief Get the currently active node.
  
  The node that is currently set as hierarchical root for new nodes is returned.
  
  \return Pointer to the CPlanNode instance that is the current hierarchy root node.

*/
CNode* CPlanLogger::activeNode() {
  return m_ndActive;
}

/*! \brief Configures the given exporter with all current plan nodes.
  
  The currently known hierarchy of plan nodes is transferred into the given exporter in order to be processed.
  
  \param expConfigure The exporter to configure with the current content of the plan event log.

*/
void CPlanLogger::configureExporter(CExporter *expConfigure) {
  // Fill up the nodes list in this exporter
  for(list<CNode*>::iterator itNode = m_lstNodes.begin();
      itNode != m_lstNodes.end();
      itNode++) {
    expConfigure->addNode(*itNode);
  }
  
  expConfigure->setDesignatorIDs(m_lstDesignatorIDs);
  expConfigure->setDesignatorEquations(m_lstDesignatorEquations);
  expConfigure->setDesignatorEquationTimes(m_lstDesignatorEquationTimes);
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

int CPlanLogger::getTimeStamp() {
  return std::time(0);
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
    this->clearNodes();
    m_ndActive = NULL;
    
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
    ROS_WARN("Did you move the package in some other folder than 'planlogger' without setting the `package-name' parameter?");
  }
  
  return bReturnvalue;
}

string CPlanLogger::experimentPath(string strExperimentName) {
  if(strExperimentName == "") {
    strExperimentName = m_strExperimentName;
  }
  
  return m_strExperimentsResultRoot + "/" + strExperimentName + "/";
}

void CPlanLogger::equateDesignators(string strMAChild, string strMAParent) {
  string strIDChild = this->getUniqueDesignatorID(strMAChild);
  string strIDParent = this->getUniqueDesignatorID(strMAParent);
  
  stringstream stsTimeEquate;
  stsTimeEquate << this->getTimeStamp();
  
  m_lstDesignatorEquations.push_back(make_pair(strIDParent, strIDChild));
  m_lstDesignatorEquationTimes.push_back(make_pair(strIDChild, stsTimeEquate.str()));
}

string CPlanLogger::getUniqueDesignatorID(string strMemoryAddress) {
  string strID = "";
  
  for(list< pair<string, string> >::iterator itPair = m_lstDesignatorIDs.begin();
      itPair != m_lstDesignatorIDs.end();
      itPair++) {
    pair<string, string> prPair = *itPair;
    
    if(prPair.first == strMemoryAddress) {
      strID = prPair.second;
      break;
    }
  }
  
  if(strID == "") {
    strID = this->generateRandomIdentifier("designator_", 14);
    m_lstDesignatorIDs.push_back(make_pair(strMemoryAddress, strID));
  }
  
  return strID;
}
