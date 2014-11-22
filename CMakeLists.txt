cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

PROJECT(kvf2d)
FIND_PACKAGE(Qt4 REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(SUITESPARESE REQUIRED)

SET(QT_USE_QTOPENGL TRUE)
INCLUDE(${QT_USE_FILE})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${SUITESPARSE_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(. ./GUI ./core ./utils ./GUI/widgets)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-reorder")

SET(kvf2d_SOURCES
	GUI/EditorWindow.cpp GUI/Main.cpp  GUI/MainWindow.cpp GUI/SidePanel.cpp GUI/AnimationPanel.cpp GUI/ThumbnailRenderer.cpp
	GUI/widgets/ActionButton.cpp
	core/MeshModel.cpp core/KVFModel.cpp core/VideoModel.cpp
	utils/cholmod_matrix.cpp utils/vector2d.cpp 
)

SET(kvf2d_HEADERS GUI/EditorWindow.h GUI/MainWindow.h GUI/SidePanel.h GUI/AnimationPanel.h GUI/ThumbnailRenderer.h GUI/widgets/ActionButton.h)
SET(kvf2d_FORMS GUI/MainWindow.ui GUI/SidePanel.ui GUI/AnimationPanel.ui)
SET(kvf2d_RESOURCES GUI/resources/resources.rc)

QT4_WRAP_CPP(kvf2d_HEADERS_MOC ${kvf2d_HEADERS})
QT4_WRAP_UI(kvf2d_FORMS_HEADERS ${kvf2d_FORMS})
QT4_ADD_RESOURCES(kvf2d_RESOURCES_RCC ${kvf2d_RESOURCES})

ADD_EXECUTABLE(kvf2d ${kvf2d_SOURCES} ${kvf2d_RESOURCES_RCC} ${kvf2d_HEADERS_MOC} ${kvf2d_FORMS_HEADERS} )

TARGET_LINK_LIBRARIES(kvf2d ${QT_LIBRARIES} ${OPENGL_LIBRARIES} ${SUITESPARSE_LIBRARIES} )
