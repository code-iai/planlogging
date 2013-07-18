#include <planlogging/CPlanNode.h>


CPlanNode::CPlanNode() {
  this->init();
  
  this->setID(-1);
  this->setName("");
}

CPlanNode::CPlanNode(int nID, string strName) {
  this->init();
  
  this->setID(nID);
  this->setName(strName);
}

CPlanNode::~CPlanNode() {
  this->clearSubnodes();
  this->clearImages();
}

void CPlanNode::init() {
  m_pnParent= NULL;
  m_bSuccess = true;
  m_nDetailLevel = 0;
}

void CPlanNode::clearSubnodes() {
  for(list<CPlanNode*>::iterator itNode = m_lstSubnodes.begin();
      itNode != m_lstSubnodes.end();
      itNode++) {
    CPlanNode* pnCurrent = *itNode;
    delete pnCurrent;
  }
  
  m_lstSubnodes.clear();
}

void CPlanNode::setName(string strName) {
  m_strName = strName;
}

string CPlanNode::name() {
  return m_strName;
}

void CPlanNode::setID(int nID) {
  m_nID = nID;
}

int CPlanNode::id() {
  return m_nID;
}

int CPlanNode::highestID() {
  int nHighestID = m_nID;
  
  for(list<CPlanNode*>::iterator itNode = m_lstSubnodes.begin();
      itNode != m_lstSubnodes.end();
      itNode++) {
    CPlanNode *pnCurrent = *itNode;
    
    nHighestID = max(nHighestID, pnCurrent->highestID());
  }
  
  return nHighestID;
}

void CPlanNode::addSubnode(CPlanNode *pnAdd) {
  m_lstSubnodes.push_back(pnAdd);
  pnAdd->setParent(this);
}

void CPlanNode::setParent(CPlanNode *pnParent) {
  m_pnParent = pnParent;
}

CPlanNode *CPlanNode::parent() {
  return m_pnParent;
}

void CPlanNode::setDescription(list<CKeyValuePair*> lstDescription) {
  for(list<CKeyValuePair*>::iterator itPair = lstDescription.begin();
      itPair != lstDescription.end();
      itPair++) {
    m_lstDescription.push_back((*itPair)->copy());
  }
}

list<CKeyValuePair*> CPlanNode::description() {
  return m_lstDescription;
}

list<CPlanNode*> CPlanNode::subnodes() {
  return m_lstSubnodes;
}

void CPlanNode::setSuccess(bool bSuccess) {
  m_bSuccess = bSuccess;
}

bool CPlanNode::success() {
  return m_bSuccess;
}

void CPlanNode::setDetailLevel(int nDetailLevel) {
  m_nDetailLevel = nDetailLevel;
}

int CPlanNode::detailLevel() {
  return m_nDetailLevel;
}

void CPlanNode::setSource(string strSource) {
  m_strSource = strSource;
}

string CPlanNode::source() {
  return m_strSource;
}

void CPlanNode::addImage(string strOrigin, string strFilename) {
  CImage *imgNew = new CImage(strOrigin, strFilename);
  m_lstImages.push_back(imgNew);
}

list<CImage*> CPlanNode::images() {
  return m_lstImages;
}

void CPlanNode::clearImages() {
  for(list<CImage*>::iterator itImage = m_lstImages.begin();
      itImage != m_lstImages.end();
      itImage++) {
    CImage *imgCurrent = *itImage;
    
    delete imgCurrent;
  }
  
  m_lstImages.clear();
}
