/*!*********************************************************************************
 *  \file       behavior_pay_attention_to_qr_code.cpp
 *  \brief      BehaviorPayAttentionToQRCode implementation file.
 *  \details    This file implements the BehaviorPayAttentionToQRCode class.
 *  \authors    Rafael Artiñano Muñoz, Guillermo Echegoyen Blanco
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
 *              All Rights Reserved
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "../include/behavior_pay_attention_to_qr_code.h"
#include <pluginlib/class_list_macros.h>
namespace attention_to_visual_markers
{
BehaviorPayAttentionToQRCode::BehaviorPayAttentionToQRCode() : BehaviorExecutionController()
{
setName("pay_attention_to_qr_code"); 
}
BehaviorPayAttentionToQRCode::~BehaviorPayAttentionToQRCode()
{

}

void BehaviorPayAttentionToQRCode::checkProgress() 
{ 

}

void BehaviorPayAttentionToQRCode::checkProcesses() 
{ 

}

void BehaviorPayAttentionToQRCode::onExecute() 
{ 

}

void BehaviorPayAttentionToQRCode::checkGoal() 
{ 

}

/*Robot Process*/
void BehaviorPayAttentionToQRCode::onConfigure()
{
  nh = getNodeHandle();
  nspace = getNamespace();

  std::string refresh;
  nh.param<std::string>("refresh_rate", refresh, DEFAULT_REFRESH_RATE);
  refresh_rate = std::stod(refresh);
  
  nh.param<std::string>("qr_interpretation_topic", qr_interpretation_str, "qr_interpretation");
}

void BehaviorPayAttentionToQRCode::onActivate()
{
  ros::ServiceClient start_controller=nh.serviceClient<std_srvs::Empty>("/"+nspace+"/qr_recognizer_process/start");
  std_srvs::Empty req;
  start_controller.call(req);

}


void BehaviorPayAttentionToQRCode::onDeactivate()
{
  ros::ServiceClient stop_controller=nh.serviceClient<std_srvs::Empty>("/"+nspace+"/qr_recognizer_process/stop");
  std_srvs::Empty req;
  stop_controller.call(req);

}

bool BehaviorPayAttentionToQRCode::checkSituation(){
  return true;
}

}
PLUGINLIB_EXPORT_CLASS(attention_to_visual_markers::BehaviorPayAttentionToQRCode, nodelet::Nodelet)
