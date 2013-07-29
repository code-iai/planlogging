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

using namespace std;


class CPlanLogger {
 private:
  list<CPlanNode*> m_lstPlanNodes;
  list<CPlanNode*> m_lstNodeList;
  CPlanNode *m_pnActive;
  bool m_bUseColor;
  string m_strExperimentsResultRoot;
  string m_strExperimentName;
  
  string replaceString(string strOriginal, string strReplaceWhat, string strReplaceBy);
  
  string generateDotDiGraph(CPlanNode *pnCurrent, int &nIndex, string strParentID, bool bSuccesses, bool bFails, int nMaxDetailLevel);
  pair<string, string> generateOWL(CPlanNode *pnCurrent, int &nIndex, string strParentID, string strPreEvent, string strPostEvent, bool bSuccesses, bool bFails, int nMaxDetailLevel);
  
  void fillPlanNodesUniqueIDs();
  string owlTypeForPlanNode(CPlanNode *pnNode);
  pair< string, list<string> > owlObjectsForPlanNode(CPlanNode *pnNode, string strNamespace);
  
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
  
  void setUseColor(bool bUseColor);
  
  string generateDotDiGraph(bool bSuccesses, bool bFails, int nMaxDetailLevel);
  string generateOWL(bool bSuccesses, bool bFails, int nMaxDetailLevel);
  
  void setExperimentsResultRoot(string strExperimentsResultRoot);
  string experimentsResultRoot();
  
  void setExperimentName(string strExperimentName);
  string experimentName();
  
  bool renewSession();
};


#endif /* __C_PLANLOGGER_H__ */
