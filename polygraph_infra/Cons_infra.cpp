/*!
 * \file Cons_infra.cpp
 * \brief Polygraph infrastructure source file for the Cons node
 * \author Alexandre Berne <alexandre.berne@cea.fr> 
 * \version 2.0
 * \date 05/07/2023
 *
 * Polygraph infrastructure source file for the Cons node - generated automatically (contact A. Berne if errors)
 * This file is dedicated to the Polygraph Infrastructure code
 * (c) CEA List - DRT/LIST/DSCIN/LCYL
 *
 */

#include "Cons.h"
#include "polygraph_trace.h"
/******************************************************/
//             USER CODE CALLBACKS                    //
/******************************************************/
extern void Cons_setup (DataStructure *datastruct);
extern void Cons_loop (DataStructure *datastruct);

/******************************************************/
//              POLYGRAPH INFRA CODE                  //
/******************************************************/
// Topic Publisher declarations

/*!
 * \struct TokenStructure
 * \brief Structure containing token values for each topics of the Cons node 
 *
 */
struct TokenStructure
{
  // Declaring token variables for each topics
  int prod_out_token = 0;
};
// Declaring TokenStructure with token values for each topics
TokenStructure Tokenstruct;

/*!
 * \struct BufferDataStructure
 * \brief Structure containing buffered topics values of the Cons node 
 *
 */
struct BufferDataStructure
{
  // Declaring variables containing buffered topics values
  // Subscribers values
  std::vector<std_msgs::String> prod_out_buffer_msg;
};
BufferDataStructure BufferDatastruct;


/*!
 * \fn int firing_node (void)
 * \brief This function is called to check if the node meeting conditions to fire the Cons_loop function
 *
 * \return The number of firing done.
 */
unsigned long Nb_jobs = 0;
int firing_node (void)
{
  int nb_firing = 0;
  while (\
  (BufferDatastruct.prod_out_buffer_msg.size() >= 1) && \
  1)
  {
    if (Nb_jobs == 0)
    {
      // Polygraph trace specify user friendly name for thread
      polytef_thread_name("Cons");
    }
    
    // Filling User Datastruct and remove it from buffer
    Datastruct.prod_out_msg = BufferDatastruct.prod_out_buffer_msg.front();
    BufferDatastruct.prod_out_buffer_msg.erase(BufferDatastruct.prod_out_buffer_msg.begin());

    // Polygraph trace start job
    polytef_job_start("Cons", Nb_jobs + 1);
    
    // Calling user loop
    Cons_loop (&Datastruct);

    // Publishing all topics
    nb_firing++;
    
    // Polygraph trace job completed
    polytef_job_finish();
    Nb_jobs++;
  }
  return (nb_firing);
}




/*!
 * \fn void prod_out_callback(const std_msgs::String::ConstPtr& msg)
 * \brief Callback function of the topic prod_out in the Cons node (prod: 1, cons: 1, init: 0)
 *
 * \param[in] msg Containing the value of the received topic
 *
 */
unsigned long Nb_prod_out_token = 0;
void prod_out_callback(const std_msgs::String::ConstPtr& msg)
{
  int once = 0;
  {
    for (int msg_loop = 0; msg_loop < 1; ++msg_loop)
    {
      // Incrementing token counter
      Tokenstruct.prod_out_token += 1;

      // Polygraph trace token reception
      if ( (((Nb_prod_out_token / 1) % 1) == 0) && (once == 0) )
      {
        // Polygraph trace reception of token
        polytef_channel_receive("prod_out", 1 + (Nb_prod_out_token / 1));
        once = 1;
      }

      if (Tokenstruct.prod_out_token % 1 == 0)
      {
        // Filling the BufferDataStructure
        BufferDatastruct.prod_out_buffer_msg.push_back(*msg);
        Tokenstruct.prod_out_token = 0;
      }
      Nb_prod_out_token++;
    }
  }
  firing_node();
}

/*!
 * \fn void init_buffer_token (void)
 * \brief Function to create valid token at initialization of Cons actor
 *
 */
void init_buffer_token (void)
{

  std_msgs::String prod_out_empty_msg;
  while (Tokenstruct.prod_out_token / 1 >= 1)
  {
    // Filling the BufferDataStructure
    BufferDatastruct.prod_out_buffer_msg.push_back(prod_out_empty_msg);
    Tokenstruct.prod_out_token -= 1;
  }
}

/*!
 * \fn int main (int argc, char **argv)
 * \brief Entry point of the Cons program
 *
 * \param[in] argc Containing number of arguments passed in the command line
 * \param[in] argv Containing arguments passed in the command line
 *
 * \return EXIT_SUCCESS - Program successfully executed.
 */
int main (int argc, char **argv)
{
  ros::init(argc, argv, "Cons_node");
  ros::NodeHandle nh;
  
  // Initializing Polygraph traces
  polytef_init("Cons");
  
  // Initializing buffer with init values
  init_buffer_token ();
  
  // Calling user setup
  Cons_setup (&Datastruct);

  // Creating Cons subscribers
  ros::Subscriber prod_out_sub = nh.subscribe("/prod_out", 1, prod_out_callback);

  // Creating Cons publishers

  // While system is OK
  ros::spin();


  // Finilize Polygraph traces
  polytef_finalize();

  return 0;
}