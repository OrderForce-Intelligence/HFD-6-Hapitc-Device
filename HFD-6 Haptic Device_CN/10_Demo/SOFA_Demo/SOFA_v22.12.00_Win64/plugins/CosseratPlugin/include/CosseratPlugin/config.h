/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture                          *
*                (c) 2006-2018 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                           Plugin Cosserat    v1.0                           *
*				                                                              *
* This plugin is also distributed under the GNU LGPL (Lesser General          *
* Public License) license with the same conditions than SOFA.                 *
*                                                                             *
* Contributors: Defrost team  (INRIA, University of Lille, CNRS,              *
*               Ecole Centrale de Lille)                                      *
*                                                                             *
* Contact information: https://project.inria.fr/softrobot/contact/            *
*                                                                             *
******************************************************************************/
#ifndef SOFA_COSSERATPLUGIN_INIT_H
#define SOFA_COSSERATPLUGIN_INIT_H

#include <sofa/helper/config.h>

#define COSSERATPLUGIN_VERSION 21.12.0

#ifdef SOFA_BUILD_COSSERATPLUGIN
#define SOFA_TARGET CosseratPlugin
#define SOFA_COSSERATPLUGIN_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
#  define SOFA_COSSERATPLUGIN_API SOFA_IMPORT_DYNAMIC_LIBRARY
#endif

/* #undef COSSERAT_PYTHON */

/** \mainpage
  This is the plugin for the Discret Cosserat Plugin
  */

#endif /* SOFA_COSSERATPLUGIN_INIT_H */
