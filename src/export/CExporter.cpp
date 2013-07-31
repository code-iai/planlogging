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

bool CExporter::writeToFile(string strContent, string strFilename) {
  if(strFilename == "") {
    strFilename = this->configuration()->stringValue("filename");
  }
  
  if(strFilename != "") {
    ofstream fsFile;
    fsFile.open(strFilename.c_str());
    fsFile << strContent;
    fsFile.close();
    
    return true;
  }
  
  return false;
}

string CExporter::replaceString(string strOriginal, string strReplaceWhat, string strReplaceBy) {
  size_t found;
  
  found = strOriginal.find(strReplaceWhat);
  while(found != string::npos) {
    strOriginal.replace(found, strReplaceWhat.length(), strReplaceBy);
    found = strOriginal.find(strReplaceWhat, found + strReplaceBy.length());
  };
  
  return strOriginal;
}

bool CExporter::nodeDisplayable(CNode* ndDisplay) {
  int nConfigMaxDetailLevel = this->configuration()->floatValue("max-detail-level");
  int nNodeDetailLevel = ndDisplay->metaInformation()->floatValue("detail-level");
  bool bDisplaySuccesses = (this->configuration()->floatValue("display-successes") == 1);
  bool bDisplayFailures = (this->configuration()->floatValue("display-failures") == 1);
  bool bNodeSuccess = (ndDisplay->metaInformation()->floatValue("success") == 1);
  
  if(nNodeDetailLevel <= nConfigMaxDetailLevel) {
    if((bNodeSuccess && bDisplaySuccesses) || (!bNodeSuccess && bDisplayFailures)) {
      return true;
    }
  }
  
  return false;
}
