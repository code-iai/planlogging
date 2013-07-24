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
  this->clearObjects();
}

void CPlanNode::init() {
  m_pnParent= NULL;
  m_bSuccess = true;
  m_nDetailLevel = 0;
  m_nStartTime = 0;
  m_nEndTime = 0;
  m_strUniqueID = "";
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
  m_nParentID = pnParent->id();
  m_pnParent = pnParent;
}

CPlanNode *CPlanNode::parent() {
  return m_pnParent;
}

int CPlanNode::parentID() {
  return m_nParentID;
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

void CPlanNode::addObject(list<CKeyValuePair*> lstDescription) {
  CObject *objNew = new CObject(lstDescription);
  m_lstObjects.push_back(objNew);
}

list<CObject*> CPlanNode::objects() {
  return m_lstObjects;
}

void CPlanNode::clearObjects() {
  for(list<CObject*>::iterator itObject = m_lstObjects.begin();
      itObject != m_lstObjects.end();
      itObject++) {
    CObject *objCurrent = *itObject;
    
    delete objCurrent;
  }
  
  m_lstObjects.clear();
}

void CPlanNode::setStartTime(int nStartTime) {
  m_nStartTime = nStartTime;
}

int CPlanNode::startTime() {
  return m_nStartTime;
}

void CPlanNode::setEndTime(int nEndTime) {
  m_nEndTime = nEndTime;
}

int CPlanNode::endTime() {
  return m_nEndTime;
}

list<int> CPlanNode::gatherTimePoints() {
  list<int> lstTimePoints;
  
  lstTimePoints.push_back(m_nStartTime);
  lstTimePoints.push_back(m_nEndTime);
  
  for(list<CPlanNode*>::iterator itNode = m_lstSubnodes.begin();
      itNode != m_lstSubnodes.end();
      itNode++) {
    list<int> lstChildTimePoints = (*itNode)->gatherTimePoints();
    
    for(list<int>::iterator itT = lstChildTimePoints.begin();
	itT != lstChildTimePoints.end();
	itT++) {
      lstTimePoints.push_back(*itT);
    }
  }
  
  return lstTimePoints;
}

void CPlanNode::setUniqueID(string strUniqueID) {
  m_strUniqueID = strUniqueID;
}

string CPlanNode::uniqueID() {
  return m_strUniqueID;
}
