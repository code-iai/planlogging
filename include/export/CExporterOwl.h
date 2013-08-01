#ifndef __C_EXPORTER_OWL_H__
#define __C_EXPORTER_OWL_H__


// System
#include <string>
#include <list>

// Private
#include <export/CExporterFileoutput.h>

using namespace std;


class CExporterOwl : public CExporterFileoutput {
 public:
  list< pair<string, string> > m_lstEntities;
  
  void addEntity(string strNickname, string strNamespace);
  
 private:
  CExporterOwl();
  ~CExporterOwl();
  
  list<string> gatherClassesForNodes(list<CNode*> lstNodes);
  list<string> gatherTimepointsForNodes(list<CNode*> lstNodes);
  
  void prepareEntities(string strNamespaceID, string strNamespace);
  string generateDocTypeBlock();
  string generateXMLNSBlock(string strNamespace);
  string generateOwlImports(string strNamespace);
  string generatePropertyDefinitions();
  string generateClassDefinitions();
  string generateEventIndividualsForNodes(list<CNode*> lstNodes);
  string generateEventIndividuals();
  string generateObjectIndividualsForNodes(list<CNode*> lstNodes);
  string generateObjectIndividuals();
  string generateTimepointIndividuals(string strNamespace);
  
  string owlClassForNode(CNode *ndNode);
  
  virtual bool runExporter(CKeyValuePair* ckvpConfigurationOverlay);
  string owlEscapeString(string strValue);
  string generateOwlStringForNodes(list<CNode*> lstNodes);
};


#endif /* __C_EXPORTER_OWL_H__ */
