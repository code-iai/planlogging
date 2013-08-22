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
#include <export/CNode.h>
#include <export/CExporter.h>

using namespace std;


class CPlanLogger {
 private:
  list<CNode*> m_lstNodes;
  list< pair<string, string> > m_lstDesignatorIDs;
  list< pair<string, string> > m_lstDesignatorEquations;
  list< pair<string, string> > m_lstDesignatorEquationTimes;
  CNode *m_ndActive;
  string m_strExperimentsResultRoot;
  string m_strExperimentName;
  
 protected:
  void setNodeAsActive(CNode *ndActive);
  CNode* activeNode();
  
  string generateRandomIdentifier(string strPrefix, unsigned int unLength);
  int getTimeStamp();
  
  string experimentPath(string strExperimentName = "");
  
 public:
  CPlanLogger();
  ~CPlanLogger();

  CNode* addNode(string strName);
  int nextFreeIndex();
  CNode* nodeForID(int nID);
  void clearNodes();
  
  void setExperimentsResultRoot(string strExperimentsResultRoot);
  string experimentsResultRoot();
  
  void setExperimentName(string strExperimentName);
  string experimentName();
  
  bool renewSession();
  
  void configureExporter(CExporter *expConfigure);
  
  string getDesignatorID(string strMemoryAddress);
  string getUniqueDesignatorID(string strMemoryAddress);
  void equateDesignators(string strMAChild, string strMAParent);
};


#endif /* __C_PLANLOGGER_H__ */
