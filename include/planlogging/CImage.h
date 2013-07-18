#ifndef __C_IMAGE_H__
#define __C_IMAGE_H__


// System
#include <string>

using namespace std;


class CImage {
 private:
  string m_strOrigin;
  string m_strFilename;
  
 public:
  CImage();
  CImage(string strOrigin, string strFilename);
  ~CImage();
  
  void setOrigin(string strOrigin);
  string origin();
  
  void setFilename(string strFilename);
  string filename();
};


#endif /* __C_IMAGE_H__ */
