
#ifndef CXX_apa_config
#define CXX_apa_config

#ifndef __cplusplus
#   error ERROR: This file requires C++ compilation (use a .cpp suffix)
#endif

#include <map>
#include <vector>
#include <mutex>
#include <string>

class apa_config
{
public:
    apa_config(std::string config_file);
    virtual ~apa_config();

    std::string getConfig(const std::string& key) const;
    const std::string& getConfigAbsolutePath() const;
    std::string getConfigAbsolutePath(const std::string& key) const;
    const std::string& getlogtype() const;
    void setlogtype(std::string keyword);
    void Destroy();

private:
    int initConfig(std::string conf_path);
    apa_config(const apa_config& x);
    apa_config& operator=(const apa_config& x);

    std::map<std::string, std::string>  m_config;
    std::string                         m_conf_path;
    std::string                         m_conf_file;
    std::string                         m_logtype;
};

#endif /* CXX_apa_config */
/* EOF */
