/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#pragma once
#include <gtsam/geometry/Pose3.h>
#include <hydra_utils/dsg_types.h>
#include <kimera_pgmo/utils/CommonStructs.h>

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
//AddMyself
#include <iostream>
#include <fstream>
#include <istream>
#include <cstdlib>
#include <ctime>

namespace hydra {

namespace lcd {

struct DsgRegistrationSolution {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool valid = false;
  NodeId from_node;
  NodeId to_node;
  gtsam::Pose3 to_T_from;
  int64_t level;
};

}  // namespace lcd

namespace incremental {

typedef std::unordered_set<NodeId> NodeIdSet;

struct SharedDsgInfo {
  using Ptr = std::shared_ptr<SharedDsgInfo>;

  SharedDsgInfo(const std::map<LayerId, char>& layer_id_map, LayerId mesh_layer_id)
      : updated(false) {
    DynamicSceneGraph::LayerIds layer_ids;
    for (const auto& id_key_pair : layer_id_map) {
      CHECK(id_key_pair.first != mesh_layer_id)
          << "Found duplicate layer id " << id_key_pair.first
          << " with mesh: " << mesh_layer_id;

      layer_ids.push_back(id_key_pair.first);
    }

    graph.reset(new DynamicSceneGraph(layer_ids, mesh_layer_id));
    latest_places.reset(new NodeIdSet);
  }

  std::mutex mutex;
  std::atomic<bool> updated;
  uint64_t last_update_time;
  DynamicSceneGraph::Ptr graph;
  std::shared_ptr<NodeIdSet> latest_places;

  std::map<NodeId, size_t> agent_key_map;
  NodeIdSet archived_places;

  std::mutex lcd_mutex;
  std::queue<lcd::DsgRegistrationSolution> loop_closures;

  //zi ji wan
  int countedges = 0;
  NodeId minID;
  NodeId tminID;
  NodeId maxID;
  NodeId maxgID;
  float Destmpx=0;
  float Destmpy=0;
  std::optional<NodeId> Parent;
  int STI =0;
  int SAI =0;
  int GainID_size=0;
  int GainID_size_last=0;
  bool HasArchived = false;
  bool StarPoi = true;
  
  // void SInit(SqStack &s)
  // {
  //   s.base=(int *)malloc(10*sizeof(int)); 
    
  //   if(!s.base) exit(-1);
  //   s.top=s.base;
  //   s.size=10;
  // }
  // int SPush(SqStack &s, int e)
  // {
  //   if(s.top-s.base>s.size)
  //   {
  //     s.base=(int *)realloc(s.base,(s.size+5)*sizeof(int)); 
  //     if(!s.base)	exit(-2);
  //     s.size=s.size+5;
  //   }
  //   *s.top=e;
  //   s.top++;
  //   return 1;
  // }
  // int SPop(SqStack &s,int &e)
  // {
  //   if(s.top==s.base)
  //     return 0;
  //   s.top--;//
  //   e=*s.top;
  //   return 1;
  // }
  // int SIsEmpty(SqStack s)
  // {
  //   if(s.top==s.base)
  //     return 1;
  //   else
  //     return 0;
  // }
  // void SDestroy(SqStack &s)
  // {
  //   free(s.base);
  // }
  
  std::map<std::map<NodeId,NodeId>,double> LINJ;
  std::map<NodeId,Eigen::Vector3d> tempIDDis;
  std::vector<std::pair<double,NodeId>> tempIDDis2;
  std::vector<std::pair<double,NodeId>> DistanceID;
  std::map<NodeId,Eigen::Vector3d> IDposition;
  std::map<NodeId,Eigen::Vector3d> IDPOtemp;
  std::vector<std::pair<double,NodeId>> DIDtemp;
  std::map<NodeId,Eigen::Vector3d> cunchu;
  std::map<NodeId,Eigen::Vector3d> allfrontier;
  std::vector<std::pair<double,NodeId>> frontier_go;
  std::vector<std::pair<double,double>> LowGainCor;
  std::map<NodeId,Eigen::Vector3d> allPlacenodes;
  std::vector<std::pair<double,NodeId>> GainID;
  std::unordered_set<NodeId> hasarrived;
  std::vector<std::pair<double,double>> HasArrivedCor;
  std::unordered_set<NodeId> lowgain_node;
  std::unordered_set<NodeId> Room;
  std::vector< std::pair<double,double> > LatestPlaceDir;
  bool hasArchived = true;
  int hasArchivedCount = 0;
  int circleCount = 0;
  int CNM = 0;
  bool circleDone = true;
  int LPDSize = 0;
  int LastLPDSize = 0;
  double wayLenth=0;
  double LenthTemp=0;
  int pgLen = 0;
  int counTime = 0;
  int ChangeRoomBool1 = 0;
  bool TimeBool1 = true;
  bool TimeBool2 = true;
  double timeDura = 0;
  bool Cauculate = false;
  clock_t start,end;
  int RoomNum = 8;
  //room exploration 
  
  bool togo_action=false;
  bool action_safe=true;
  bool ruminate = false;
  bool rumi = false;
  bool TurnReady = false;
  bool circleReady = true;
  bool stepReady = false;
  int steps=0;
  int stare = 0;
  int roomding=0;
  int waittime=0;
  int AddComSize = 0;
  std::unordered_set<NodeId> hasarrived_true;
  std::map<NodeId,Eigen::Vector3d> allROOMfrontier;
  std::map<NodeId,Eigen::Vector3d> allROOMnodes;
  std::map<NodeId,Eigen::Vector3d> ROOM1nodes;
  std::map<NodeId,Eigen::Vector3d> ROOM2nodes;
  std::map<NodeId,Eigen::Vector3d> ROOM3nodes;
  std::map<NodeId,Eigen::Vector3d> ROOM4nodes;
  std::map<NodeId,Eigen::Vector3d> ROOM5nodes;
  std::map<NodeId,Eigen::Vector3d> ROOM6nodes;
  std::map<NodeId,Eigen::Vector3d> ROOM7nodes;
  std::map<NodeId,Eigen::Vector3d> ROOM8nodes;
  std::map<NodeId,Eigen::Vector3d> ROOM9nodes;
  std::vector<std::pair<double,NodeId>> ROOM_frontier_go;
  std::vector<double> location_x, location_y;
  //room exploration
  //auto room exploration
  std::unordered_set<int> hasarrived_room;
  std::vector<double> dis_temp;
  std::map<int,std::map<NodeId,Eigen::Vector3d>> ROOMnodes;
  std::vector<std::pair<double,int>> ROOMselect;
  bool initializing=true;
  //auto room exploration

  // std::vector<std::pair<double,NodeId>> Gaintemp;
  std::vector<double> Min_dist;
  std::vector<int> allnodepath;
  std::vector<NodeId> pathIDstack;
  bool go1=true;
  bool GO=true;
  bool Went=true;
  bool PushDone=true;
  bool PubBool=true;
  bool went_go=false;
  bool gainachived=false;
  bool delemax=false;
  int VideoCount = 0;
};

struct DsgBackendStatus {
  size_t total_loop_closures_;
  size_t new_loop_closures_;
  size_t total_factors_;
  size_t total_values_;
  size_t new_factors_;
  size_t new_graph_factors_;
  size_t trajectory_len_;

  void reset() {
    total_loop_closures_ = 0;
    new_loop_closures_ = 0;
    total_factors_ = 0;
    total_values_ = 0;
    new_factors_ = 0;
    new_graph_factors_ = 0;
    trajectory_len_ = 0;
  }
};

}  // namespace incremental
}  // namespace hydra
