#ifndef __C_EXPORTER_H__
#define __C_EXPORTER_H__


// System
#include <string>
#include <list>
#include <fstream>

// Other
#include <designators/CKeyValuePair.h>

// Private
#include <export/CNode.h>

using namespace std;


class CExporter {
 private:
  list<CNode*> m_lstNodes;
  list< pair<string, string> > m_lstDesignatorIDs;
  list< pair<string, string> > m_lstDesignatorEquations;
  
  CKeyValuePair* m_ckvpConfiguration;
  
  void renewUniqueIDsForNode(CNode *ndRenew);
  
 protected:
  list< pair<string, string> > m_lstDesignatorEquationTimes;
  
 public:
  CExporter();
  ~CExporter();
  
  CKeyValuePair* configuration();
  
  void addNode(CNode *ndAdd);
  list<CNode*> nodes();
  
  void clearNodes();
  
  virtual bool runExporter(CKeyValuePair* ckvpConfigurationOverlay);
  
  string generateRandomIdentifier(string strPrefix, unsigned int unLength);
  string generateUniqueID(string strPrefix, unsigned int unLength);
  bool uniqueIDPresent(string strUniqueID);
  
  void renewUniqueIDs();
  
  virtual string nodeIDPrefix(CNode* ndInQuestion, string strProposition);
  
  string replaceString(string strOriginal, string strReplaceWhat, string strReplaceBy);
  virtual bool nodeDisplayable(CNode* ndDisplay);
  
  void setDesignatorIDs(list< pair<string, string> > lstDesignatorIDs);
  void setDesignatorEquations(list< pair<string, string> > lstDesignatorEquations);
  void setDesignatorEquationTimes(list< pair<string, string> > lstDesignatorEquationTimes);  
  
  list<string> designatorIDs();
  list<string> parentDesignatorsForID(string strID);
  list<string> successorDesignatorsForID(string strID);
  string equationTimeForSuccessorID(string strID);
};


#endif /* __C_EXPORTER_H__ */
