#ifndef __C_PLANLOGGER_H__
#define __C_PLANLOGGER_H__


// System
#include <string>
#include <list>
#include <sstream>
#include <fstream>
#include <ctime>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>

// Private
#include <planlogging/CPlanNode.h>
#include <export/CExporter.h>

using namespace std;


class CPlanLogger {
 private:
  list<CPlanNode*> m_lstPlanNodes;
  list<CPlanNode*> m_lstNodeList;
  CPlanNode *m_pnActive;
  string m_strExperimentsResultRoot;
  string m_strExperimentName;
  
  string replaceString(string strOriginal, string strReplaceWhat, string strReplaceBy);
  
  void fillPlanNodesUniqueIDs();
  
 protected:
  void setNodeAsActive(CPlanNode *pnActive);
  CPlanNode* activeNode();

  string generateRandomIdentifier(string strPrefix, unsigned int unLength);
  int getTimeStamp();
  
  string experimentPath(string strExperimentName = "");
  
 public:
  CPlanLogger();
  ~CPlanLogger();

  CPlanNode* addPlanNode(string strName);
  int nextFreeIndex();
  CPlanNode* planNodeForID(int nID);
  void clearPlanNodes();
  
  void setExperimentsResultRoot(string strExperimentsResultRoot);
  string experimentsResultRoot();
  
  void setExperimentName(string strExperimentName);
  string experimentName();
  
  bool renewSession();
  
  CNode* convertPlanNodeToNode(CPlanNode* pnConvert);
  void configureExporter(CExporter *expConfigure);
};


#endif /* __C_PLANLOGGER_H__ */
