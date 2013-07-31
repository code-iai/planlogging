#include <export/CExporterDot.h>


CExporterDot::CExporterDot() {
}

CExporterDot::~CExporterDot() {
}

void CExporterDot::setOutputFilename(string strFilename) {
  this->configuration()->setValue("filename", strFilename);
}

string CExporterDot::outputFilename() {
  return this->configuration()->stringValue("filename");
}

bool CExporterDot::runExporter(CKeyValuePair* ckvpConfigurationOverlay) {
  this->renewUniqueIDs();
  
  if(this->outputFilename() != "") {
    string strGraphID = this->generateRandomIdentifier("plangraph_", 8);
    string strToplevelID = this->generateUniqueID("node_", 8);
    
    string strDot = "digraph " + strGraphID + " {\n";
    
    strDot += "  " + strToplevelID + " [shape=doublecircle, style=bold, label=\"top-level\"];\n";
    
    bool bSuccess;
    strDot += this->generateDotStringForNodes(this->nodes(), strToplevelID, bSuccess);
    
    strDot += "}\n";
    
    return this->writeToFile(strDot);
  }
  
  return false;
}

string CExporterDot::generateDotStringForNodes(list<CNode*> lstNodes, string strParentID, bool& bSuccess) {
  string strDot = "";
  
  for(list<CNode*>::iterator itNode = lstNodes.begin();
      itNode != lstNodes.end();
      itNode++) {
    CNode *ndCurrent = *itNode;
    string strNodeID = ndCurrent->uniqueID();
    
    string strFillColor;
    string strEdgeColor;
    if(ndCurrent->metaInformation()->floatValue("success") == 1) {
      strFillColor = "#ddffdd";
      strEdgeColor = "green";
    } else {
      strFillColor = "#ffdddd";
      strEdgeColor = "red";
    }
    
    string strParameters = "";
    
    list<CKeyValuePair*> lstDescription = ndCurrent->description();
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
	
	strValue = this->dotEscapeString(strValue);
	strParameters += "|{" + this->dotEscapeString(ckvpCurrent->key()) + " | " + strValue + "}";
      }
    }
    
    string strLabel = "{" + this->dotEscapeString(ndCurrent->title()) + strParameters + "}";
    
    strDot += "\n  " + strNodeID + " [shape=Mrecord, style=filled, fillcolor=\"" + strFillColor + "\", label=\"" + strLabel + "\"];\n";
    strDot += "  edge [color=\"" + strEdgeColor + "\"];\n";
    strDot += "  " + strParentID + " -> " + strNodeID + ";\n";
    
    bSuccess = true;
  }
  
  return strDot;
}

string CExporterDot::dotEscapeString(string strValue) {
  strValue = this->replaceString(strValue, "\n", "\\n");
  strValue = this->replaceString(strValue, "{", "\\{");
  strValue = this->replaceString(strValue, "}", "\\}");
  strValue = this->replaceString(strValue, "<", "\\<");
  strValue = this->replaceString(strValue, ">", "\\>");
  strValue = this->replaceString(strValue, "\"", "\\\"");
  strValue = this->replaceString(strValue, "|", "\\|");
  
  return strValue;
}
