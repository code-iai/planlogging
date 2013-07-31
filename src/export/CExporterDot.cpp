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
    
    strDot += this->generateDotStringForNodes(this->nodes(), strToplevelID);
    
    strDot += "}\n";
    
    return this->writeToFile(strDot);
  }
  
  return false;
}

string CExporterDot::generateDotStringForNodes(list<CNode*> lstNodes, string strParentID) {
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
	} else if(ckvpCurrent->type() == POSE) {
	  strValue = "pose (?)";
	} else if(ckvpCurrent->type() == POSESTAMPED) {
	  strValue = "pose stamped (?)";
	}
	
	strValue = this->dotEscapeString(strValue);
	strParameters += "|{" + this->dotEscapeString(ckvpCurrent->key()) + " | " + strValue + "}";
      }
    }
    
    string strLabel = "{" + this->dotEscapeString(ndCurrent->title()) + strParameters + "}";
    
    strDot += "\n  " + strNodeID + " [shape=Mrecord, style=filled, fillcolor=\"" + strFillColor + "\", label=\"" + strLabel + "\"];\n";
    strDot += "  edge [color=\"" + strEdgeColor + "\"];\n";
    strDot += "  " + strParentID + " -> " + strNodeID + ";\n";
    
    // Images
    strDot += this->generateDotImagesStringForNode(ndCurrent);
    
    // Objects
    strDot += this->generateDotObjectsStringForNode(ndCurrent);
    
    // Subnodes
    strDot += this->generateDotStringForNodes(ndCurrent->subnodes(), strNodeID);
  }
  
  return strDot;
}

string CExporterDot::generateDotImagesStringForNode(CNode *ndImages) {
  string strDot = "";
  
  CKeyValuePair *ckvpImages = ndImages->metaInformation()->childForKey("images");
  list<CKeyValuePair*> lstChildren = ckvpImages->children();
  
  unsigned int unIndex = 0;
  for(list<CKeyValuePair*>::iterator itChild = lstChildren.begin();
      itChild != lstChildren.end();
      itChild++, unIndex++) {
    CKeyValuePair *ckvpChild = *itChild;
    
    string strOrigin = ckvpChild->stringValue("origin");
    string strFilename = ckvpChild->stringValue("filename");
    
    stringstream sts;
    sts << ndImages->uniqueID() << "_image_" << unIndex;
    
    strDot += "  " + sts.str() + " [shape=box, label=\"" + strOrigin + "\", width=\"6cm\", height=\"6cm\", fixedsize=true, imagescale=true, image=\"" + strFilename + "\"];\n";
    strDot += "  " + sts.str() + " -> " + ndImages->uniqueID() + ";\n";
  }
  
  return strDot;
}

string CExporterDot::generateDotObjectsStringForNode(CNode *ndObjects) {
  string strDot = "";
  
  CKeyValuePair *ckvpObjects = ndObjects->metaInformation()->childForKey("objects");
  list<CKeyValuePair*> lstChildren = ckvpObjects->children();
  
  unsigned int unIndex = 0;
  for(list<CKeyValuePair*>::iterator itChild = lstChildren.begin();
      itChild != lstChildren.end();
      itChild++, unIndex++) {
    CKeyValuePair *ckvpChild = *itChild;
    
    stringstream sts;
    sts << ndObjects->uniqueID() << "_image_" << unIndex;
    
    strDot += "  " + sts.str() + " [shape=box, label=\"some object\"];\n";
    strDot += "  " + sts.str() + " -> " + ndObjects->uniqueID() + ";\n";
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
