#ifndef __C_EXPORTER_OWL_H__
#define __C_EXPORTER_OWL_H__


// System
#include <string>
#include <list>

// Private
#include <export/CExporterFileoutput.h>

using namespace std;


class CExporterOwl : public CExporterFileoutput {
 private:
  list< pair<string, string> > m_lstEntities;
  
  void addEntity(string strNickname, string strNamespace);
  
 public:
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
  string generateEventIndividualsForNodes(list<CNode*> lstNodes, string strNamespace);
  string generateEventIndividuals(string strNamespace);
  string generateObjectIndividualsForNodes(list<CNode*> lstNodes, string strNamespace);
  string generateObjectIndividuals(string strNamespace);
  string generateTimepointIndividuals(string strNamespace);
  
  string owlClassForNode(CNode *ndNode);
  string owlClassForObject(CKeyValuePair *ckvpObject);  
  virtual string nodeIDPrefix(CNode* ndInQuestion, string strProposition);
  
  virtual bool runExporter(CKeyValuePair* ckvpConfigurationOverlay);
  string owlEscapeString(string strValue);
  string generateOwlStringForNodes(list<CNode*> lstNodes);
};


#endif /* __C_EXPORTER_OWL_H__ */
