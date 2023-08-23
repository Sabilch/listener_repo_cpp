/*!
 * \file Cons_user.cpp
 * \brief User source file for the Cons node
 * \author Alexandre Berne <alexandre.berne@cea.fr> 
 * \version 1.0
 * \date 05/07/2023
 *
 * User source file for the Cons node - generated automatically (contact A. Berne if errors)
 * This file is dedicated to the application/user code
 * (c) CEA List - DRT/LIST/DSCIN/LCYL
 *
 */

#include "Cons.h"

DataStructure Datastruct;

/*!
 * \fn void Cons_setup (DataStructure *datastruct)
 * \brief Cons_setup is the init function for the user code
 *
 * \param[in, out] datastruct Structure containing all topic values
 */
void Cons_setup (DataStructure *datastruct)
{
}

/*!
 * \fn void Cons_loop (DataStructure *datastruct)
 * \brief Cons_loop is the function called every time the node is firing
 *
 * \param datastruct[in, out] Structure containing all topic values
 */
void Cons_loop (DataStructure *datastruct)
{
    std::cout <<"consumer here"<< std::endl;
}