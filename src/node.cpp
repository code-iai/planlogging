#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <list>

#include <planlogger/StartTask.h>
#include <planlogger/StopTask.h>
#include <planlogger/ExtractTask.h>

using namespace std;


struct Task {
  int id;
  string name;
  int parameter_count;
};


list<struct Task> lstTasks;
int nGlobalUniqueID;


class CTask {
private:
  list<CTask*> m_lstSubTasks;
  string m_strName;
  int m_nID;
  CTask *m_tskParent;
  list< pair<string, string> > m_lstParameters;

public:
  CTask(CTask *tskParent, int nID, string strName, list< pair<string, string> > lstParameters) {
    m_tskParent = tskParent;
    m_nID = nID;
    m_strName = strName;
    m_lstParameters = lstParameters;
  }
  
  void addChild(CTask *tskAdd) {
    m_lstSubTasks.push_back(tskAdd);
  }
  
  CTask* parent() {
    return m_tskParent;
  }
  
  string stringID() {
    stringstream sts;
    sts << "n";
    sts << m_nID;
    
    return sts.str();
  }
  
  string stringParams() {
    string strResult = "";
    
    for(list< pair<string, string> >::iterator itPair = m_lstParameters.begin();
	itPair != m_lstParameters.end();
	itPair++) {
      strResult += "|{" + (*itPair).first + " | " + (*itPair).second + "}";
    }
    
    if(strResult != "") {
      strResult += "|";
    }
    
    return strResult;
  }
  
  string name() {
    return m_strName;
  }
  
  string generateDiGraph() {
    string strReturn = "";
    string strID = this->stringID();
    
    if(m_tskParent == NULL) {
      // This is the root node, so generate the proper surrounding
      // brackets and all.
      strReturn += "digraph plangraph {\n";
      
      // Introduce yourself. Only the root node does this as all
      // subsequent nodes have been introduces by someone else above
      // them.
      
      strReturn += "  " + strID + " [shape=doublecircle, label=\"" + m_strName + this->stringParams() + "\"];\n";
    }
    
    // Introduce all children
    for(list<CTask*>::iterator itChild = m_lstSubTasks.begin();
	itChild != m_lstSubTasks.end();
	itChild++) {
      CTask *tskChild = *itChild;
      
      strReturn += "  " + tskChild->stringID() + " [shape=Mrecord, label=\"|{" + tskChild->name() + tskChild->stringParams() + "}|\"];\n";
    }
    
    // Connect all lines to the children
    for(list<CTask*>::iterator itChild = m_lstSubTasks.begin();
	itChild != m_lstSubTasks.end();
	itChild++) {
      CTask *tskChild = *itChild;
      
      strReturn += "  " + strID + " -> " + tskChild->stringID() + ";\n";
    }

    // Insert all children's graph parts
    for(list<CTask*>::iterator itChild = m_lstSubTasks.begin();
	itChild != m_lstSubTasks.end();
	itChild++) {
      CTask *tskChild = *itChild;
      
      strReturn += tskChild->generateDiGraph();
    }
    
    if(m_tskParent == NULL) {
      strReturn += "}\n";
    }
    
    return strReturn;
  }
};


CTask *tskRoot;
CTask *tskActive;


struct Task taskForID(int id) {
  for(list<struct Task>::iterator itTask = lstTasks.begin();
      itTask != lstTasks.end();
      itTask++) {
    struct Task tsk = *itTask;
    
    if(tsk.id == id) {
      return tsk;
    }
  }
  
  struct Task tsk;
  tsk.id = -1;
  
  return tsk;
}

bool taskIDExists(int id) {
  struct Task tsk = taskForID(id);
  
  return tsk.id == id;
}

bool removeTask(int id) {
  for(list<struct Task>::iterator itTask = lstTasks.begin();
      itTask != lstTasks.end();
      itTask++) {
    struct Task tsk = *itTask;
    
    if(tsk.id == id) {
      lstTasks.erase(itTask);
      tskActive = tskActive->parent();
      
      return true;
    }
  }
  
  return false;
}

string taskName(int id) {
  struct Task tsk = taskForID(id);
  
  if(tsk.id == id) {
    return tsk.name;
  }
  
  return "";
}

int addTask(string name, list< pair<string, string> > lstParameters) {
  struct Task tsk;
  tsk.name = name;
  tsk.id = 1; // First task gets the number 1. This is the lowest task
	      // number.
  
  do {
    if(!taskIDExists(tsk.id)) {
      break;
    }
    
    tsk.id++;
  } while(true);
  
  lstTasks.push_back(tsk);
  
  CTask *tskNew = new CTask(tskActive, nGlobalUniqueID, tsk.name, lstParameters);
  tskActive->addChild(tskNew);
  tskActive = tskNew;
  nGlobalUniqueID++;
  
  return tsk.id;
}

string replaceString(string strOriginal, string strReplaceWhat, string strReplaceBy) {
  size_t found;
  
  found = strOriginal.find(strReplaceWhat);
  while(found != string::npos) {
    strOriginal.replace(found, strReplaceWhat.length(), strReplaceBy);
    found = strOriginal.find(strReplaceWhat, found + strReplaceBy.length());
  };
  
  return strOriginal;
}

bool startTaskCB(planlogger::StartTask::Request &req, planlogger::StartTask::Response &res) {
  list< pair<string, string> > lstParameters;
  
  for(int nI = 0; nI < min(req.param_names.size(), req.param_values.size()); nI++) {
    string strName = string(req.param_names[nI]);
    strName = replaceString(strName, "\"", "\\\"");
    strName = replaceString(strName, "\n", "\\n");
    string strValue = string(req.param_values[nI]);
    strValue = replaceString(strValue, "\"", "\\\"");
    strValue = replaceString(strValue, "\n", "\\n");
    
    pair<string, string> prStrings = make_pair(strName, strValue);
    lstParameters.push_back(prStrings);
  }
  
  int id = addTask(req.name, lstParameters);
  res.id = id;
  
  cout << "Started task: " << req.name << " (id = " << id << "):" << endl;
  for(list< pair<string, string> >::iterator itPair = lstParameters.begin();
      itPair != lstParameters.end();
      itPair++) {
    cout << " - " << (*itPair).first << " = " << (*itPair).second << endl;
  }
  
  return true;
}

bool stopTaskCB(planlogger::StopTask::Request &req, planlogger::StopTask::Response &res) {
  if(taskIDExists(req.id)) {
    struct Task tsk = taskForID(req.id);
    
    cout << "Stopped task: " << tsk.name  << " (id = " << req.id << ")" << endl;
    
    return removeTask(tsk.id);
  }
  
  return false;
}

bool extractTaskCB(planlogger::ExtractTask::Request &req, planlogger::ExtractTask::Response &res) {
  string strFileContents = tskRoot->generateDiGraph();
  
  if(req.filename != "") {
    ofstream myfile;
    myfile.open(req.filename.c_str());
    myfile << strFileContents;
    myfile.close();
    
    return true;
  }
  
  return false;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "planlogger");
  ros::NodeHandle nh;
  
  list< pair<string, string> > lstEmpty;
  tskRoot = new CTask(NULL, 0, "root", lstEmpty);
  tskActive = tskRoot;
  nGlobalUniqueID = 1;
  
  ros::ServiceServer srvStartTask = nh.advertiseService("start_task", startTaskCB);
  ros::ServiceServer srvStopTask = nh.advertiseService("stop_task", stopTaskCB);
  ros::ServiceServer srvExtractTask = nh.advertiseService("extract_task", extractTaskCB);
  
  ros::spin();
  
  return 0;
}
