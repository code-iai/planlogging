#include <export/CExporterOwl.h>


CExporterOwl::CExporterOwl() {
}

CExporterOwl::~CExporterOwl() {
}

void CExporterOwl::prepareEntities(string strNamespaceID, string strNamespace) {
  m_lstEntities.clear();
  
  this->addEntity("owl", "http://www.w3.org/2002/07/owl#");
  this->addEntity("xsd", "http://www.w3.org/2001/XMLSchema#");
  this->addEntity("knowrob", "http://ias.cs.tum.edu/kb/knowrob.owl#");
  this->addEntity("rdfs", "http://www.w3.org/2000/01/rdf-schema#");
  this->addEntity("rdf", "http://www.w3.org/1999/02/22-rdf-syntax-ns#");
  this->addEntity(strNamespaceID, strNamespace + "#");
}

void CExporterOwl::addEntity(string strNickname, string strNamespace) {
  m_lstEntities.push_back(make_pair(strNickname, strNamespace));
}

string CExporterOwl::generateDocTypeBlock() {
  string strDot = "<!DOCTYPE rdf:RDF [\n";
  
  for(list< pair<string, string> >::iterator itPair = m_lstEntities.begin();
      itPair != m_lstEntities.end();
      itPair++) {
    pair<string, string> prEntity = *itPair;
    
    strDot += "    <!ENTITY " + prEntity.first + " \"" + prEntity.second + "\" >\n";
  }
  
  strDot += "]>\n\n";
  return strDot;
}

string CExporterOwl::generateXMLNSBlock(string strNamespace) {
  string strDot = "<rdf:RDF xmlns=\"" + strNamespace + "#\"\n";
  strDot += "     xml:base=\"" + strNamespace + "\"\n";
  
  for(list< pair<string, string> >::iterator itPair = m_lstEntities.begin();
      itPair != m_lstEntities.end();
      itPair++) {
    pair<string, string> prEntity = *itPair;
    
    if(itPair != m_lstEntities.begin()) {
      strDot += "\n";
    }

    strDot += "     xmlns:" + prEntity.first + "=\"" + prEntity.second + "\"";
  }

  strDot += ">\n\n";
  return strDot;
}

string CExporterOwl::generateOwlImports(string strNamespace) {
  string strDot = "";
  
  strDot += "    <owl:Ontology rdf:about=\"" + strNamespace + "\">\n";
  strDot += "        <owl:imports rdf:resource=\"http://ias.cs.tum.edu/kb/knowrob.owl\"/>\n";
  strDot += "    </owl:Ontology>\n\n";

  return strDot;
}

string CExporterOwl::generatePropertyDefinitions() {
  string strDot = "    <!-- Property Definitions -->\n\n";
  
  list<string> lstProperties;
  lstProperties.push_back("http://ias.cs.tum.edu/kb/knowrob.owl#startTime");
  lstProperties.push_back("http://ias.cs.tum.edu/kb/knowrob.owl#endTime");
  lstProperties.push_back("http://ias.cs.tum.edu/kb/knowrob.owl#preEvent");
  lstProperties.push_back("http://ias.cs.tum.edu/kb/knowrob.owl#postEvent");
  lstProperties.push_back("http://ias.cs.tum.edu/kb/knowrob.owl#subAction");
  lstProperties.push_back("http://ias.cs.tum.edu/kb/knowrob.owl#detectedObject");
  lstProperties.push_back("http://ias.cs.tum.edu/kb/knowrob.owl#objectActedOn");
  
  for(list<string>::iterator itProperty = lstProperties.begin();
      itProperty != lstProperties.end();
      itProperty++) {
    strDot += "    <owl:ObjectProperty rdf:about=\"" + *itProperty + "\"/>\n\n";
  }
  
  return strDot;
}

list<string> CExporterOwl::gatherClassesForNodes(list<CNode*> lstNodes) {
  list<string> lstClasses;
  
  for(list<CNode*>::iterator itNode = lstNodes.begin();
      itNode != lstNodes.end();
      itNode++) {
    CNode *ndCurrent = *itNode;
    
    list<string> lstClassesSubnodes = this->gatherClassesForNodes(ndCurrent->subnodes());
    lstClassesSubnodes.push_back(this->owlClassForNode(ndCurrent));
    
    for(list<string>::iterator itClassSubnode = lstClassesSubnodes.begin();
	itClassSubnode != lstClassesSubnodes.end();
	itClassSubnode++) {
      bool bExists = false;
      
      for(list<string>::iterator itClassNode = lstClasses.begin();
	  itClassNode != lstClasses.end();
	  itClassNode++) {
	if(*itClassSubnode == *itClassNode) {
	  bExists = true;
	  break;
	}
      }
      
      if(!bExists) {
	lstClasses.push_back(*itClassSubnode);
      }
    }
  }
  
  return lstClasses;
}

list<string> CExporterOwl::gatherTimepointsForNodes(list<CNode*> lstNodes) {
  list<string> lstTimepoints;
  
  for(list<CNode*>::iterator itNode = lstNodes.begin();
      itNode != lstNodes.end();
      itNode++) {
    CNode *ndCurrent = *itNode;
    
    list<string> lstTimepointsSubnodes = this->gatherTimepointsForNodes(ndCurrent->subnodes());
    int nStartTime = ndCurrent->metaInformation()->floatValue("time-start");
    int nEndTime = ndCurrent->metaInformation()->floatValue("time-end");
    stringstream stsStart;
    stsStart << nStartTime;
    stringstream stsEnd;
    stsEnd << nEndTime;
    
    lstTimepointsSubnodes.push_back(stsStart.str());
    lstTimepointsSubnodes.push_back(stsEnd.str());
    
    for(list<string>::iterator itTimepointSubnode = lstTimepointsSubnodes.begin();
	itTimepointSubnode != lstTimepointsSubnodes.end();
	itTimepointSubnode++) {
      bool bExists = false;
      
      for(list<string>::iterator itTimepointNode = lstTimepoints.begin();
	  itTimepointNode != lstTimepoints.end();
	  itTimepointNode++) {
	if(*itTimepointSubnode == *itTimepointNode) {
	  bExists = true;
	  break;
	}
      }
      
      if(!bExists) {
	lstTimepoints.push_back(*itTimepointSubnode);
      }
    }
  }
  
  return lstTimepoints;
}

string CExporterOwl::generateClassDefinitions() {
  string strDot = "    <!-- Class Definitions -->\n\n";
  
  list<string> lstClasses = this->gatherClassesForNodes(this->nodes());
  lstClasses.push_back("&knowrob;Timepoint");
  
  for(list<string>::iterator itClass = lstClasses.begin();
      itClass != lstClasses.end();
      itClass++) {
    strDot += "    <owl:Class rdf:about=\"" + *itClass + "\"/>\n\n";
  }
  
  return strDot;
}

string CExporterOwl::generateEventIndividualsForNodes(list<CNode*> lstNodes) {
  string strDot = "";
  
  // Implement this
  
  return strDot;
}

string CExporterOwl::generateEventIndividuals() {
  string strDot = "    <!-- Event Individuals -->\n\n";
  strDot += this->generateEventIndividualsForNodes(this->nodes());
  
  return strDot;
}

string CExporterOwl::generateObjectIndividualsForNodes(list<CNode*> lstNodes) {
  string strDot = "";
  
  // Implement this
  
  return strDot;
}

string CExporterOwl::generateObjectIndividuals() {
  string strDot = "    <!-- Object Individuals -->\n\n";
  strDot += this->generateObjectIndividualsForNodes(this->nodes());
  
  return strDot;
}

string CExporterOwl::generateTimepointIndividuals(string strNamespace) {
  string strDot = "    <!-- Timepoint Individuals -->\n\n";
  
  list<string> lstTimepoints = this->gatherTimepointsForNodes(this->nodes());
  for(list<string>::iterator itTimepoint = lstTimepoints.begin();
      itTimepoint != lstTimepoints.end();
      itTimepoint++) {
    strDot += "    <owl:NamedIndividual rdf:about=\"&" + strNamespace + ";timepoint_" + *itTimepoint + "\">\n";
    strDot += "        <rdf:type rdf:resource=\"&knowrob;TimePoint\"/>\n";
    strDot += "    </owl:NamedIndividual>\n\n";
  }
  
  return strDot;
}

string CExporterOwl::owlClassForNode(CNode *ndNode) {
  string strName = ndNode->title();
  string strReturnvalue = "&knowrob;CRAMAction";
  
  if(strName == "WITH-DESIGNATORS") {
    // Is this right? Or is there a more fitting type for that?
    strReturnvalue = "&knowrob;WithDesignators";
  } else if(strName.substr(0, 5) == "GOAL-") {
    // This is a goal definition.
    string strGoal = strName.substr(5);
    
    // Missing yet:
    /*
      PREVENT
      MAINTAIN
      INFORM (speech act, add information to belief state from outside)
     */
    
    if(strGoal == "PERCEIVE-OBJECT") {
      strReturnvalue = "&knowrob;Perceive";
    } else if(strGoal == "ACHIEVE") {
      strReturnvalue = "&knowrob;Achieve";
    } else if(strGoal == "PERFORM") { // Should go into another structure (?)
      strReturnvalue = "&knowrob;Perform";
    } else if(strGoal == "MONITOR-ACTION") {
      strReturnvalue = "&knowrob;Monitor";
    } else if(strGoal == "PERFORM-ON-PROCESS-MODULE") {
      strReturnvalue = "&knowrob;PerformOnProcessModule";
    } else {
      strReturnvalue = "&knowrob;DeclarativeGoal";
    }
  } else if(strName.substr(0, 8) == "RESOLVE-") {
    // This is a designator resolution.
    string strDesigType = strName.substr(8);
    
    if(strDesigType == "LOCATION-DESIGNATOR") {
      strReturnvalue = "&knowrob;ResolveLocationDesignator";
    } else if(strDesigType == "ACTION-DESIGNATOR") {
      strReturnvalue = "&knowrob;ResolveActionDesignator";
    }
  } else if(strName.substr(0, 21) == "REPLACEABLE-FUNCTION-") {
    // This is an internal function name
    string strFunction = strName.substr(21);
    
    if(strFunction == "NAVIGATE") {
      strReturnvalue = "&knowrob;Navigate";
    }
  } else if(strName == "UIMA-PERCEIVE") {
    strReturnvalue = "&knowrob;VisualPerception";
  } else if(strName == "AT-LOCATION") {
    strReturnvalue = "&knowrob;AtLocation";
  }
  
  return strReturnvalue;
}

bool CExporterOwl::runExporter(CKeyValuePair* ckvpConfigurationOverlay) {
  this->renewUniqueIDs();
  
  if(this->outputFilename() != "") {
    string strOwl = "<?xml version=\"1.0\"?>\n\n";
    string strNamespaceID = this->generateRandomIdentifier("namespace_", 8);
    string strNamespace = "http://ias.cs.tum.edu/kb/" + strNamespaceID;
    
    // Prepare content
    this->prepareEntities(strNamespaceID, strNamespace);
    
    // Assemble OWL source
    strOwl += this->generateDocTypeBlock();
    strOwl += this->generateXMLNSBlock(strNamespace);
    strOwl += this->generateOwlImports(strNamespace);
    strOwl += this->generatePropertyDefinitions();
    strOwl += this->generateClassDefinitions();
    strOwl += this->generateEventIndividuals();
    strOwl += this->generateObjectIndividuals();
    strOwl += this->generateTimepointIndividuals(strNamespaceID);
    strOwl += "</rdf:RDF>\n";
    
    // Write the .owl file
    return this->writeToFile(strOwl);
  }
  
  return false;
}

string CExporterOwl::owlEscapeString(string strValue) {
  return strValue;
}

string CExporterOwl::generateOwlStringForNodes(list<CNode*> lstNodes) {
  string strOwl = "";
  
  // Generate the OWL source here
  
  return strOwl;
}
