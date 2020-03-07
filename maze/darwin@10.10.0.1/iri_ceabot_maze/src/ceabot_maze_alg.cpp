#include "ceabot_maze_alg.h"

CeabotMazeAlgorithm::CeabotMazeAlgorithm(void)
{
  pthread_mutex_init(&this->access_,NULL);
}

CeabotMazeAlgorithm::~CeabotMazeAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void CeabotMazeAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=config;
  
  this->unlock();
}

// CeabotMazeAlgorithm Public API
