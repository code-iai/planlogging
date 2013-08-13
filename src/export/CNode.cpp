#include <export/CNode.h>


CNode::CNode() {
  this->init();
}

CNode::CNode(string strTitle) {
  this->init();
  this->setTitle(strTitle);
}

CNode::CNode(list<CKeyValuePair*> lstDescription) {
  this->init();
  this->setDescription(lstDescription);
}

CNode::~CNode() {
  this->clearSubnodes();
  this->clearDescription();
}

void CNode::init() {
  m_strTitle = "";
  m_ckvpMetaInformation = new CKeyValuePair();
  m_ndParent = NULL;
  m_nID = 0;
}

void CNode::setDescription(list<CKeyValuePair*> lstDescription) {
  for(list<CKeyValuePair*>::iterator itPair = lstDescription.begin();
      itPair != lstDescription.end();
      itPair++) {
    CKeyValuePair *ckvpPair = *itPair;
    
    m_lstDescription.push_back(ckvpPair->copy());
  }
}

void CNode::clearDescription() {
  for(list<CKeyValuePair*>::iterator itPair = m_lstDescription.begin();
      itPair != m_lstDescription.end();
      itPair++) {
    CKeyValuePair *ckvpPair = *itPair;
      
    delete ckvpPair;
  }
  
  m_lstDescription.clear();
}

void CNode::clearSubnodes() {
  for(list<CNode*>::iterator itNode = m_lstSubnodes.begin();
      itNode != m_lstSubnodes.end();
      itNode++) {
    CNode *ndCurrent = *itNode;
    
    delete ndCurrent;
  }
  
  m_lstSubnodes.clear();
}

list<CKeyValuePair*> CNode::description() {
  return m_lstDescription;
}

void CNode::setTitle(string strTitle) {
  m_strTitle = strTitle;
}

string CNode::title() {
  return m_strTitle;
}

void CNode::addSubnode(CNode* ndAdd) {
  ndAdd->setParent(this);
  m_lstSubnodes.push_back(ndAdd);
}

list<CNode*> CNode::subnodes() {
  return m_lstSubnodes;
}

void CNode::setUniqueID(string strUniqueID) {
  m_strUniqueID = strUniqueID;
}

string CNode::uniqueID() {
  return m_strUniqueID;
}

bool CNode::includesUniqueID(string strUniqueID) {
  bool bReturnvalue = false;
  
  if(m_strUniqueID == strUniqueID) {
    bReturnvalue = true;
  } else {
    for(list<CNode*>::iterator itNode = m_lstSubnodes.begin();
	itNode != m_lstSubnodes.end();
	itNode++) {
      CNode *ndCurrent = *itNode;
      
      if(ndCurrent->includesUniqueID(strUniqueID)) {
	bReturnvalue = true;
	break;
      }
    }
  }
  
  return bReturnvalue;
}

CKeyValuePair *CNode::metaInformation() {
  return m_ckvpMetaInformation;
}

void CNode::setID(int nID) {
  m_nID = nID;
}

int CNode::id() {
  return m_nID;
}

int CNode::highestID() {
  int nHighestID = m_nID;
  
  for(list<CNode*>::iterator itNode = m_lstSubnodes.begin();
      itNode != m_lstSubnodes.end();
      itNode++) {
    CNode *ndCurrent = *itNode;
    
    nHighestID = max(nHighestID, ndCurrent->highestID());
  }
  
  return nHighestID;
}

void CNode::setParent(CNode* ndParent) {
  m_ndParent = ndParent;
}

CNode* CNode::parent() {
  return m_ndParent;
}

void CNode::setPrematurelyEnded(bool bPrematurelyEnded) {
  this->metaInformation()->setValue(string("prematurely-ended"), (bPrematurelyEnded ? 1 : 0));
}

bool CNode::prematurelyEnded() {
  return (this->metaInformation()->floatValue("prematurely-ended") == 0 ? false : true);
}

void CNode::addImage(string strOrigin, string strFilename) {
  CKeyValuePair* ckvpImages = this->metaInformation()->addChild("images");
  
  stringstream sts;
  sts << "image-";
  sts << ckvpImages->children().size();
  
  CKeyValuePair* ckvpImage = ckvpImages->addChild(sts.str());
  ckvpImage->setValue(string("origin"), strOrigin);
  ckvpImage->setValue(string("filename"), strFilename);
}

void CNode::addObject(list<CKeyValuePair*> lstDescription) {
  CKeyValuePair* ckvpObjects = this->metaInformation()->addChild("objects");
  
  stringstream sts;
  sts << "object-";
  sts << ckvpObjects->children().size();
  
  CKeyValuePair* ckvpObject = ckvpObjects->addChild(sts.str());
  for(list<CKeyValuePair*>::iterator itPair = lstDescription.begin();
      itPair != lstDescription.end();
      itPair++) {
    ckvpObject->addChild((*itPair)->copy());
  }
}
