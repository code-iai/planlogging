#include <planlogging/CImage.h>


CImage::CImage() {
  this->setOrigin("");
  this->setFilename("");
}

CImage::CImage(string strOrigin, string strFilename) {
  this->setOrigin(strOrigin);
  this->setFilename(strFilename);
}

CImage::~CImage() {
}

void CImage::setOrigin(string strOrigin) {
  m_strOrigin = strOrigin;
}

string CImage::origin() {
  return m_strOrigin;
}

void CImage::setFilename(string strFilename) {
  m_strFilename = strFilename;
}

string CImage::filename() {
  return m_strFilename;
}
