#include <planlogging/CPlanLogger.h>


CPlanLogger::CPlanLogger() {
  m_pnActive = NULL;
  this->setUseColor(true);
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

string CPlanLogger::generateDotDiGraph(bool bSuccesses, bool bFails, int nMaxDetailLevel) {
  string strReturnvalue = "digraph plangraph {\n";
  int nIndex = 1;
  
  strReturnvalue += "  n0 [shape=doublecircle, style=bold, label=\"root\"];\n";
  
  for(list<CPlanNode*>::iterator itNode = m_lstPlanNodes.begin();
      itNode != m_lstPlanNodes.end();
      itNode++) {
    CPlanNode *pnCurrent = *itNode;
    
    strReturnvalue += this->generateDotDiGraph(pnCurrent, nIndex, "n0", bSuccesses, bFails, nMaxDetailLevel);
  }
  
  strReturnvalue += "}\n";
  
  return strReturnvalue;
}

void CPlanLogger::setUseColor(bool bUseColor) {
  m_bUseColor = bUseColor;
}

string CPlanLogger::generateDotDiGraph(CPlanNode *pnCurrent, int &nIndex, string strParentID, bool bSuccesses, bool bFails, int nMaxDetailLevel) {
  string strReturnvalue = "";
  
  if(((bSuccesses && pnCurrent->success()) || (bFails && !pnCurrent->success())) && (pnCurrent->detailLevel() <= nMaxDetailLevel)) {
    stringstream sts;
    sts << "n";
    sts << nIndex++;
    string strID = sts.str();
  
    // Prepare the parameter string
    string strParameters = "";
    list<CKeyValuePair*> lstDescription = pnCurrent->description();
  
    for(list<CKeyValuePair*>::iterator itPair = lstDescription.begin();
	itPair != lstDescription.end();
	itPair++) {
      CKeyValuePair *ckvpCurrent = *itPair;
    
      if(ckvpCurrent->key().at(0) != '_') {
	string strValue = "?";
	if(ckvpCurrent->type() == STRING) {
	  strValue = ckvpCurrent->stringValue();
	} else if(ckvpCurrent->type() == FLOAT) {
	  stringstream sts;
	  sts << ckvpCurrent->floatValue();
	  strValue = sts.str();
	}
    
	strValue = this->replaceString(strValue, "\n", "\\n");
	strValue = this->replaceString(strValue, "{", "\\{");
	strValue = this->replaceString(strValue, "}", "\\}");
	strValue = this->replaceString(strValue, "<", "\\<");
	strValue = this->replaceString(strValue, ">", "\\>");
	strValue = this->replaceString(strValue, "\"", "\\\"");
	strValue = this->replaceString(strValue, "|", "\\|/");
    
	strParameters += "|{" + ckvpCurrent->key() + " | " + strValue + "}";
      }
    }
  
    // Introduce yourself and your affiliation
    string strColor = "black";
    string strBackgroundColor = "white";
    if(m_bUseColor) {
      if(pnCurrent->success()) {
	strColor = "green";
	strBackgroundColor = "#ddffdd";
      } else {
	strColor = "red";
	strBackgroundColor = "#ffdddd";
      }
    }
    
    strReturnvalue += "  " + strID + " [shape=Mrecord, style=filled, fillcolor=\"" + strBackgroundColor + "\", label=\"{" + pnCurrent->name() + strParameters + "}\"];\n";
    
    strReturnvalue += "  edge [color=" + strColor + "];\n";
    strReturnvalue += "  " + strParentID + " -> " + strID + ";\n";
  
    list<CPlanNode*> lstSubnodes = pnCurrent->subnodes();
    for(list<CPlanNode*>::iterator itNode = lstSubnodes.begin();
	itNode != lstSubnodes.end();
	itNode++) {
      CPlanNode *pnNode = *itNode;
    
      strReturnvalue += this->generateDotDiGraph(pnNode, nIndex, strID, bSuccesses, bFails, nMaxDetailLevel);
    }
    
    // Draw associated images and connect them to the graph
    list<CImage*> lstImages = pnCurrent->images();
    if(lstImages.size() > 0) {
      strReturnvalue += "  edge [color=black];\n";
      int nImageIndex = 0;
      
      for(list<CImage*>::iterator itImage = lstImages.begin();
	  itImage != lstImages.end();
	  itImage++) {
	CImage *imgImage = *itImage;
	
	stringstream sts;
	sts << strID;
	sts << "_image";
	sts << nImageIndex;
	string strImageID = sts.str();
	
	strReturnvalue += "  " + strImageID + " [shape=box, label=\"" + imgImage->origin() + "\", width=\"6cm\", height=\"6cm\", fixedsize=true, imagescale=true, image=\"" + imgImage->filename() + "\"];\n";
	strReturnvalue += "  " + strImageID + " -> " + strID + ";\n";
	
	nImageIndex++;
      }
    }
    
    // Insert associated objects and connect them to the graph
    list<CObject*> lstObjects = pnCurrent->objects();
    if(lstObjects.size() > 0) {
      strReturnvalue += "  edge [color=black];\n";
      int nObjectIndex = 0;
      
      for(list<CObject*>::iterator itObject = lstObjects.begin();
	  itObject != lstObjects.end();
	  itObject++) {
	CObject *objObject = *itObject;
	
	stringstream sts;
	sts << strID;
	sts << "_object";
	sts << nObjectIndex;
	string strObjectID = sts.str();
	
	strReturnvalue += "  " + strObjectID + " [shape=box, label=\"some object\"];\n";
	strReturnvalue += "  " + strObjectID + " -> " + strID + ";\n";
	
	nObjectIndex++;
      }
    }
  }
  
  return strReturnvalue;
}

string CPlanLogger::generateOWL(bool bSuccesses, bool bFails, int nMaxDetailLevel) {
  string strReturnvalue = "<owl:some-event-file>\n";
  int nIndex = 1;
  
  strReturnvalue += "  <owl:namedEvent name=\"event0\">\n";
  strReturnvalue += "    <!-- Some root event properties here -->\n";
  strReturnvalue += "  </owl:namedEvent>\n\n";
  
  for(list<CPlanNode*>::iterator itNode = m_lstPlanNodes.begin();
      itNode != m_lstPlanNodes.end();
      itNode++) {
    CPlanNode *pnCurrent = *itNode;
    
    strReturnvalue += this->generateOWL(pnCurrent, nIndex, "event0", bSuccesses, bFails, nMaxDetailLevel);
  }
  
  strReturnvalue += "</owl:some-event-file>\n";
  
  return strReturnvalue;
}

string CPlanLogger::generateOWL(CPlanNode *pnCurrent, int &nIndex, string strParentID, bool bSuccesses, bool bFails, int nMaxDetailLevel) {
  string strReturnvalue = "";
  
  if(((bSuccesses && pnCurrent->success()) || (bFails && !pnCurrent->success())) && (pnCurrent->detailLevel() <= nMaxDetailLevel)) {
    stringstream sts;
    sts << "event";
    sts << nIndex++;
    string strID = sts.str();
  
    // Prepare the parameter string
    string strParameters = "";
    list<CKeyValuePair*> lstDescription = pnCurrent->description();
  
    for(list<CKeyValuePair*>::iterator itPair = lstDescription.begin();
	itPair != lstDescription.end();
	itPair++) {
      CKeyValuePair *ckvpCurrent = *itPair;
    
      if(ckvpCurrent->key().at(0) != '_') {
	string strValue = "?";
	if(ckvpCurrent->type() == STRING) {
	  strValue = ckvpCurrent->stringValue();
	} else if(ckvpCurrent->type() == FLOAT) {
	  stringstream sts;
	  sts << ckvpCurrent->floatValue();
	  strValue = sts.str();
	}
    
	strValue = this->replaceString(strValue, "\n", "\\n");
	strValue = this->replaceString(strValue, "<", "\\<");
	strValue = this->replaceString(strValue, ">", "\\>");
	strValue = this->replaceString(strValue, "\"", "\\\"");
	
	strParameters += "    <owl:" + ckvpCurrent->key() + ">" + strValue + "</owl:" + ckvpCurrent->key() + ">\n";
      }
    }
    
    // Introduce yourself and your affiliation
    strReturnvalue += "  <owl:namedEvent name=\"" + strID + "\">\n";
    strReturnvalue += strParameters;
    strReturnvalue += "    <owl:parentEvent>" + strParentID + "</owl:parentEvent>\n";
    strReturnvalue += "    <owl:label>" + pnCurrent->name() + "</owl:label>\n";
    strReturnvalue += "  </owl:namedEvent>\n\n";
    
    list<CPlanNode*> lstSubnodes = pnCurrent->subnodes();
    for(list<CPlanNode*>::iterator itNode = lstSubnodes.begin();
	itNode != lstSubnodes.end();
	itNode++) {
      CPlanNode *pnNode = *itNode;
    
      strReturnvalue += this->generateOWL(pnNode, nIndex, strID, bSuccesses, bFails, nMaxDetailLevel);
    }
  }
  
  return strReturnvalue;
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
