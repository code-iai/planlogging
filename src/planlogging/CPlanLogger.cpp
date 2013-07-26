#include <planlogging/CPlanLogger.h>


CPlanLogger::CPlanLogger() {
  m_pnActive = NULL;
  this->setUseColor(true);

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

string CPlanLogger::generateOWL(bool bSuccesses, bool bFails, int nMaxDetailLevel) {
  string strReturnvalue = "<?xml version=\"1.0\"?>\n\n";
  int nIndex = 1;
  
  // Renew the node's unique IDs.
  this->fillPlanNodesUniqueIDs();
  
  string strNamespaceID = this->generateRandomIdentifier("", 8);
  string strNamespaceToken = "execution-log-" + strNamespaceID;
  string strNamespace = "http://ias.cs.tum.edu/kb/" + strNamespaceToken;
  
  list< pair<string, string> > lstEntities;
  lstEntities.push_back(make_pair("owl", "http://www.w3.org/2002/07/owl#"));
  lstEntities.push_back(make_pair("xsd", "http://www.w3.org/2001/XMLSchema#"));
  lstEntities.push_back(make_pair("knowrob", "http://ias.cs.tum.edu/kb/knowrob.owl#"));
  lstEntities.push_back(make_pair("rdfs", "http://www.w3.org/2000/01/rdf-schema#"));
  lstEntities.push_back(make_pair("rdf", "http://www.w3.org/1999/02/22-rdf-syntax-ns#"));
  lstEntities.push_back(make_pair(strNamespaceToken, strNamespace + "#"));
  
  strReturnvalue += "<!DOCTYPE rdf:RDF [\n";
  for(list< pair<string, string> >::iterator itPair = lstEntities.begin();
      itPair != lstEntities.end();
      itPair++) {
    pair<string, string> prEntity = *itPair;
    
    strReturnvalue += "    <!ENTITY " + prEntity.first + " \"" + prEntity.second + "\" >\n";
  }
  strReturnvalue += "]>\n\n";
  
  strReturnvalue += "<rdf:RDF xmlns=\"" + strNamespace + "#\"\n";
  strReturnvalue += "     xml:base=\"" + strNamespace + "\"\n";
  for(list< pair<string, string> >::iterator itPair = lstEntities.begin();
      itPair != lstEntities.end();
      itPair++) {
    pair<string, string> prEntity = *itPair;
    
    if(itPair != lstEntities.begin()) {
      strReturnvalue += "\n";
    }
    
    strReturnvalue += "     xmlns:" + prEntity.first + "=\"" + prEntity.second + "\"";
  }
  
  strReturnvalue += ">\n\n";
  
  strReturnvalue += "    <owl:Ontology rdf:about=\"" + strNamespace + "\">\n";
  strReturnvalue += "        <owl:imports rdf:resource=\"http://ias.cs.tum.edu/kb/knowrob.owl\"/>\n";
  strReturnvalue += "    </owl:Ontology>\n\n";
  
  strReturnvalue += "    <!-- Object Properties -->\n\n";
  
  list<string> lstProperties;
  lstProperties.push_back("http://ias.cs.tum.edu/kb/knowrob.owl#startTime");
  lstProperties.push_back("http://ias.cs.tum.edu/kb/knowrob.owl#endTime");
  lstProperties.push_back("http://ias.cs.tum.edu/kb/knowrob.owl#preEvent");
  lstProperties.push_back("http://ias.cs.tum.edu/kb/knowrob.owl#postEvent");
  lstProperties.push_back("http://ias.cs.tum.edu/kb/knowrob.owl#subAction");
  
  for(list<string>::iterator itProperty = lstProperties.begin();
      itProperty != lstProperties.end();
      itProperty++) {
    strReturnvalue += "    <owl:ObjectProperty rdf:about=\"" + *itProperty + "\"/>\n\n";
  }
  
  strReturnvalue += "    <!-- Class Definitions -->\n\n";
  
  list<string> lstClasses;
  lstClasses.push_back("&knowrob;TimePoint");
  for(list<CPlanNode*>::iterator itNode = m_lstNodeList.begin();
      itNode != m_lstNodeList.end();
      itNode++) {
    string strType = this->owlTypeForPlanNode(*itNode);
    bool bExists = false;
    
    for(list<string>::iterator itClass = lstClasses.begin();
	itClass != lstClasses.end();
	itClass++) {
      if(*itClass == strType) {
	bExists = true;
	break;
      }
    }
    
    if(!bExists) {
      lstClasses.push_back(strType);
    }
  }
  
  for(list<string>::iterator itClass = lstClasses.begin();
      itClass != lstClasses.end();
      itClass++) {
    // Why is the namespace for e.g. classes extended in the .owl
    // files? Is it a problem when we don't do it here? It should be
    // okay, though.
    strReturnvalue += "    <owl:Class rdf:about=\"" + *itClass + "\"/>\n\n";
  }
  
  strReturnvalue += "    <!-- Event Individuals -->\n\n";
  
  for(list<CPlanNode*>::iterator itNode = m_lstPlanNodes.begin();
      itNode != m_lstPlanNodes.end();
      itNode++) {
    CPlanNode *pnCurrent = *itNode;
    
    string strPreEventTemp = "";
    if(itNode != m_lstPlanNodes.begin()) {
      list<CPlanNode*>::iterator itTemp = itNode;
      itTemp--;
      strPreEventTemp = (*itTemp)->uniqueID();
    }
    
    string strPostEventTemp = "";
    if(itNode != m_lstPlanNodes.end()) {
      list<CPlanNode*>::iterator itTemp = itNode;
      itTemp++;
      
      if(itTemp != m_lstPlanNodes.end()) {
	strPostEventTemp = (*itTemp)->uniqueID();
      }
    }
    
    pair<string, string> prChildResult = this->generateOWL(pnCurrent, nIndex, strNamespaceToken, strPreEventTemp, strPostEventTemp, bSuccesses, bFails, nMaxDetailLevel);
    strReturnvalue += prChildResult.second;
  }

  strReturnvalue += "    <!-- Object Individuals -->\n\n";
  
  for(list<CPlanNode*>::iterator itNode = m_lstNodeList.begin();
      itNode != m_lstNodeList.end();
      itNode++) {
    CPlanNode *pnCurrent = *itNode;
    
    pair< string, list<string> > prObjects = this->owlObjectsForPlanNode(pnCurrent, strNamespaceToken);
    strReturnvalue += prObjects.first;
  }
  
  strReturnvalue += "    <!-- Timepoint Individuals -->\n\n";
  
  list<int> lstTimepoints;
  list<int> lstPointsPresent;
  for(list<CPlanNode*>::iterator itNode = m_lstPlanNodes.begin();
      itNode != m_lstPlanNodes.end();
      itNode++) {
    list<int> lstT = (*itNode)->gatherTimePoints();
    
    for(list<int>::iterator itT = lstT.begin();
	itT != lstT.end();
	itT++) {
      lstTimepoints.push_back(*itT);
    }
  }
  
  for(list<int>::iterator itT = lstTimepoints.begin();
      itT != lstTimepoints.end();
      itT++) {
    bool bPresent = false;
    for(list<int>::iterator itT2 = lstPointsPresent.begin();
	itT2 != lstPointsPresent.end();
	itT2++) {
      if(*itT == *itT2) {
	bPresent = true;
	break;
      }
    }
    
    if(!bPresent) {
      stringstream sts;
      sts << *itT;
      
      strReturnvalue += "    <owl:NamedIndividual rdf:about=\"&" + strNamespaceToken + ";timepoint_" + sts.str() + "\">\n";
      strReturnvalue += "        <rdf:type rdf:resource=\"&knowrob;TimePoint\"/>\n";
      strReturnvalue += "    </owl:NamedIndividual>\n\n";
      
      lstPointsPresent.push_back(*itT);
    }
  }
  
  strReturnvalue += "</rdf:RDF>\n";
  
  return strReturnvalue;
}

pair<string, string> CPlanLogger::generateOWL(CPlanNode *pnCurrent, int &nIndex, string strParentID, string strPreEvent, string strPostEvent, bool bSuccesses, bool bFails, int nMaxDetailLevel) {
  string strReturnvalue = "";
  string strID = "";
  
  if(((bSuccesses && pnCurrent->success()) || (bFails && !pnCurrent->success())) && (pnCurrent->detailLevel() <= nMaxDetailLevel)) {
    strID = pnCurrent->uniqueID();
    
    int nStartTime = pnCurrent->startTime();
    int nEndTime = pnCurrent->endTime();
    
    stringstream stsStartTime;
    stsStartTime << nStartTime;

    stringstream stsEndTime;
    stsEndTime << nEndTime;
    
    list<CPlanNode*> lstSubnodes = pnCurrent->subnodes();
    list<string> lstChildIDs;
    for(list<CPlanNode*>::iterator itNode = lstSubnodes.begin();
	itNode != lstSubnodes.end();
	itNode++) {
      CPlanNode *pnNode = *itNode;
      
      string strPreEventTemp = "";
      if(itNode != lstSubnodes.begin()) {
	list<CPlanNode*>::iterator itTemp = itNode;
	itTemp--;
	strPreEventTemp = (*itTemp)->uniqueID();
      }
      
      string strPostEventTemp = "";
      if(itNode != lstSubnodes.end()) {
	list<CPlanNode*>::iterator itTemp = itNode;
	itTemp++;
	
	if(itTemp != lstSubnodes.end()) {
	  strPostEventTemp = (*itTemp)->uniqueID();
	}
      }
      
      pair<string, string> prChild = this->generateOWL(pnNode, nIndex, strParentID, strPreEventTemp, strPostEventTemp, bSuccesses, bFails, nMaxDetailLevel);
      strReturnvalue += prChild.second;
      lstChildIDs.push_back(prChild.first);
    }
    
    // Introduce yourself and your affiliation
    strReturnvalue += "    <owl:namedIndividual rdf:about=\"&" + strParentID + ";" + strID + "\">\n";
    strReturnvalue += "        <rdf:type rdf:resource=\"" + this->owlTypeForPlanNode(pnCurrent) + "\"/>\n";
    strReturnvalue += "        <knowrob:startTime rdf:resource=\"&" + strParentID + ";timepoint_" + stsStartTime.str() + "\"/>\n";
    strReturnvalue += "        <knowrob:endTime rdf:resource=\"&" + strParentID + ";timepoint_" + stsEndTime.str() + "\"/>\n";
    
    if(strPreEvent != "") {
      strReturnvalue += "        <knowrob:preEvent rdf:resource=\"&" + strParentID + ";" + strPreEvent + "\"/>\n";
    }
    
    if(strPostEvent != "") {
      strReturnvalue += "        <knowrob:postEvent rdf:resource=\"&" + strParentID + ";" + strPostEvent + "\"/>\n";
    }
    
    for(list<string>::iterator itID = lstChildIDs.begin();
	itID != lstChildIDs.end();
	itID++) {
      strReturnvalue += "        <knowrob:subAction rdf:resource=\"&" + strParentID + ";" + *itID + "\"/>\n";
    }
    
    list<CObject*> lstObjects = pnCurrent->objects();
    for(list<CObject*>::iterator itObject = lstObjects.begin();
	itObject != lstObjects.end();
	itObject++) {
      CObject *objObject = *itObject;
      
      if(pnCurrent->name() == "UIMA-PERCEIVE") {
	strReturnvalue += "        <knowrob:detectedObject rdf:resource=\"&" + strParentID + ";" + objObject->uniqueID() + "\"/>\n";
      }
    }
    
    strReturnvalue += "    </owl:namedIndividual>\n\n";
  }
  
  return make_pair(strID, strReturnvalue);
}

string CPlanLogger::owlTypeForPlanNode(CPlanNode *pnNode) {
  string strName = pnNode->name();
  string strReturnvalue = "&knowrob;Thing";
  
  if(strName == "WITH-DESIGNATORS") {
    // Is this right? Or is there a more fitting type for that?
    strReturnvalue = "&knowrob;SoftwareObject";
  } else if(strName.substr(0, 5) == "GOAL-") {
    // This is a goal definition.
    string strGoal = strName.substr(5);
    
    if(strGoal == "PERCEIVE-OBJECT") {
      //strReturnvalue = "&knowrob;VisualPerception";
    } else if(strGoal == "ACHIEVE") {
    } else if(strGoal == "PERFORM") {
      strReturnvalue = "&knowrob;Action";
    } else if(strGoal == "MONITOR-ACTION") {
    } else if(strGoal == "PERFORM-ON-PROCESS-MODULE") {
    }
  } else if(strName.substr(0, 8) == "RESOLVE-") {
    // This is a designator resolution.
    string strDesigType = strName.substr(8);
    
    if(strDesigType == "LOCATION-DESIGNATOR") {
    } else if(strDesigType == "ACTION-DESIGNATOR") {
    }
  } else if(strName.substr(0, 21) == "REPLACEABLE-FUNCTION-") {
    // This is an internal function name
    string strFunction = strName.substr(21);
  } else if(strName == "UIMA-PERCEIVE") {
    strReturnvalue = "&knowrob;VisualPerception";
  } else {
    strReturnvalue = "&knowrob;Event";
  }
  
  return strReturnvalue;
}

pair< string, list<string> > CPlanLogger::owlObjectsForPlanNode(CPlanNode *pnNode, string strNamespace) {
  string strName = pnNode->name();
  string strReturnvalue = "";
  list<string> lstIDs;
  
  if(strName == "UIMA-PERCEIVE") {
    list<CObject*> lstObjects = pnNode->objects();
    
    for(list<CObject*>::iterator itObject = lstObjects.begin();
	itObject != lstObjects.end();
	itObject++) {
      CObject *objObject = *itObject;
      
      strReturnvalue += "    <owl:namedIndividual rdf:about=\"&" + strNamespace + ";" + objObject->uniqueID() + "\">\n";
      strReturnvalue += "        <rdf:type rdf:resource=\"&knowrob;Thing\"/>\n";
      strReturnvalue += "    </owl:namedIndividual>\n\n";
      
      lstIDs.push_back(objObject->uniqueID());
    }
  }
  
  return make_pair(strReturnvalue, lstIDs);
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
