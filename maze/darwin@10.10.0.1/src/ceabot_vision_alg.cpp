#include "ceabot_vision_alg.h"

CeabotVisionAlgorithm::CeabotVisionAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

CeabotVisionAlgorithm::~CeabotVisionAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void CeabotVisionAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// CeabotVisionAlgorithm Public API
