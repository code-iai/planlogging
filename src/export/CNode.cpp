#include <export/CNode.h>


CNode::CNode() {
  this->init();
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
}

void CNode::setDescription(list<CKeyValuePair*> lstDescription) {
  for(list<CKeyValuePair*>::iterator itPair = m_lstDescription.begin();
      itPair != m_lstDescription.end();
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
