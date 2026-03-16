/*!
 * @file Utilities.cpp
 * @brief Common utility functions
 */

#include <ctime>
#include <iomanip>

#include <utilities.h>

/*!
 * Write std::string to file with given name
 */
void writeStringToFile(const std::string &fileName,
                       const std::string &fileData)
{
  FILE *fp = fopen(fileName.c_str(), "w");
  if (!fp)
  {
    printf("Failed to fopen %s\n", fileName.c_str());
    throw std::runtime_error("Failed to open file");
  }
  fprintf(fp, "%s", fileData.c_str());
  fclose(fp);
}

/*!
 * Get the current time and date as a string
 */
std::string getCurrentTimeAndDate()
{
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream ss;
  ss << std::put_time(&tm, "%c");
  return ss.str();
}

/*!
 * Todo: do something better to keep track of where we are relative to the
 * config directory
 */
std::string getConfigDirectoryPath() { return "../config/"; }

/*!
 * Get the LCM URL with desired TTL.
 */
std::string getLcmUrl(s64 ttl)
{
  assert(ttl >= 0 && ttl <= 255);
  return "udpm://239.255.76.67:7667?ttl=" + std::to_string(ttl);
}

void cleaning_file(std::string _file_name, std::string &_ret_file)
{
  _ret_file = _file_name;
  _ret_file += ".txt";

  std::list<std::string>::iterator iter =
      std::find(gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
  if (gs_fileName_string.end() == iter)
  {
    gs_fileName_string.push_back(_file_name);
    remove(_ret_file.c_str());
  }
}
