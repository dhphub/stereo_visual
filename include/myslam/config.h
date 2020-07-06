//
// Created by dhp on 5/05/19.
//

#ifndef CONFIG_H
#define CONFIG_H
#include <myslam/common_include.h>

namespace myslam
{
class Config 
{
public:
  static std::shared_ptr<Config> config_;
  cv::FileStorage file_;
  Config() {};
  ~Config();
  static void setParameterFile( const std::string& filename );
  template< typename T >
  static T get( const std::string& key )
  {
    return T( Config::config_->file_[key] );
  }
};
  
}
#endif
