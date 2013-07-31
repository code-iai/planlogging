#include <export/CExporter.h>


CExporter::CExporter() {
  m_ckvpConfiguration = new CKeyValuePair();
}

CExporter::~CExporter() {
  this->clearNodes();
  delete m_ckvpConfiguration;
}

void CExporter::addNode(CNode *ndAdd) {
  m_lstNodes.push_back(ndAdd);
}

list<CNode*> CExporter::nodes() {
  return m_lstNodes;
}

void CExporter::clearNodes() {
  for(list<CNode*>::iterator itNode = m_lstNodes.begin();
      itNode != m_lstNodes.end();
      itNode++) {
    delete *itNode;
  }
  
  m_lstNodes.clear();
}

CKeyValuePair* CExporter::configuration() {
  return m_ckvpConfiguration;
}

string CExporter::nodeIDPrefix(CNode* ndInQuestion, string strProposition) {
  // NOTE(winkler): Override this function in subsequent subclasses to
  // decide on unique ID prefixes according to what the content of
  // `ndInQuestion' is. In this basic implementation, the default
  // proposition 1strProposition' is always accepted.
  
  return strProposition;
}

void CExporter::renewUniqueIDsForNode(CNode *ndRenew) {
  string strNodeIDPrefix = this->nodeIDPrefix(ndRenew, "node_");
  ndRenew->setUniqueID(this->generateUniqueID(strNodeIDPrefix, 8));
  
  list<CNode*> lstSubnodes = ndRenew->subnodes();
  for(list<CNode*>::iterator itNode = lstSubnodes.begin();
      itNode != lstSubnodes.end();
      itNode++) {
    this->renewUniqueIDsForNode(*itNode);
  }
}

void CExporter::renewUniqueIDs() {
  for(list<CNode*>::iterator itNode = m_lstNodes.begin();
      itNode != m_lstNodes.end();
      itNode++) {
    this->renewUniqueIDsForNode(*itNode);
  }
}

bool CExporter::runExporter(CKeyValuePair* ckvpConfigurationOverlay) {
  // NOTE(winkler): This is a dummy, superclass exporter. It does not
  // actually export anything. Subclass it to get *actual*
  // functionality.
  
  return true;
}

string CExporter::generateRandomIdentifier(string strPrefix, unsigned int unLength) {
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

string CExporter::generateUniqueID(string strPrefix, unsigned int unLength) {
  string strID;
  
  do {
    strID = this->generateRandomIdentifier(strPrefix, unLength);
  } while(this->uniqueIDPresent(strID));
  
  return strID;
}

bool CExporter::uniqueIDPresent(string strUniqueID) {
  for(list<CNode*>::iterator itNode = m_lstNodes.begin();
      itNode != m_lstNodes.end();
      itNode++) {
    if((*itNode)->includesUniqueID(strUniqueID)) {
      return true;
    }
  }
  
  return false;
}
