cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

PROJECT(kvf2d)
FIND_PACKAGE(Qt4 REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
SET(QT_USE_QTOPENGL TRUE)
INCLUDE(${QT_USE_FILE})

###################################################################################################

if (WIN32)
	SET(ROOT C:/Users/mlevtsky/Workspace)
	SET (EXTERNAL_LIBRARIES
		${ROOT}/suitesparse/amd.lib
		${ROOT}/suitesparse/camd.lib
		${ROOT}/suitesparse/colamd.lib
		${ROOT}/suitesparse/blas.lib
		${ROOT}/suitesparse/cholmod.lib
		${ROOT}/suitesparse/lapack.lib
		
		${ROOT}/ffmpeg/lib/avcodec.lib
		${ROOT}/ffmpeg/lib/avformat.lib
		${ROOT}/ffmpeg/lib/avutil.lib
		${ROOT}/ffmpeg/lib/swscale.lib
	)
	
	SET(EXTERNAL_INCLUDES
		${ROOT}/suitesparse/include/
		${ROOT}/ffmpeg/include/
	)

	SET(ICON_RC_FILE GUI/icon/winicon.rc)

	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -D_CRT_SECURE_NO_WARNINGS -D_CRT_NONSTDC_NO_DEPRECATE  -DNOMINMAX -D_USE_MATH_DEFINES ")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /OPT:NOREF" )
	
endif(WIN32)

###################################################################################################

if (APPLE)
	SET(ROOT _something_)
	SET (EXTERNAL_LIBRARIES
		${ROOT}/suitesparse/amd.dylib
		${ROOT}/suitesparse/camd.dylib
		${ROOT}/suitesparse/colamd.dylib
		${ROOT}/suitesparse/blas.dylib
		${ROOT}/suitesparse/cholmod.dylib
		${ROOT}/suitesparse/lapack.dylib
		
		${ROOT}/ffmpeg/lib/avcodec.dylib
		${ROOT}/ffmpeg/lib/avformat.dylib
		${ROOT}/ffmpeg/lib/avutil.dylib
		${ROOT}/ffmpeg/lib/swscale.dylib
	)	
	SET(EXTERNAL_INCLUDES
		${ROOT}/suitesparse/include/
		${ROOT}/ffmpeg/include/
	)	
endif(APPLE)

###################################################################################################

if (NOT APPLE AND UNIX) # <- this is the way I like it
	SET (EXTERNAL_LIBRARIES
		cholmod amd camd colamd
		openblas lapack		
		avcodec avformat avutil swscale
	)

	SET (EXTERNAL_INCLUDES
		/usr/include/suitesparse
	)

	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-reorder")
endif(NOT APPLE AND UNIX)

###################################################################################################

INCLUDE_DIRECTORIES(
	${CMAKE_CURRENT_BINARY_DIR} 
	${EXTERNAL_INCLUDES}
	. ./GUI ./core ./utils ./GUI/widgets
)

SET(kvf2d_SOURCES
	GUI/EditorWindow.cpp GUI/Main.cpp  GUI/MainWindow.cpp 
	GUI/SidePanel.cpp GUI/AnimationPanel.cpp GUI/OffScreenRenderer.cpp
	GUI/widgets/ActionButton.cpp GUI/ProgramState.cpp
	GUI/QVideoEncoder.cpp

	core/MeshModel.cpp core/KVFModel.cpp core/VideoModel.cpp
	core/BDMORPH.cpp core/OutlineModel.cpp

	utils/cholmod_matrix.cpp utils/cholmod_common.cpp utils/vector2d.cpp 
	utils/triangle.c utils/utils.cpp
)

SET(kvf2d_HEADERS_QT
	GUI/EditorWindow.h GUI/MainWindow.h GUI/SidePanel.h 
	GUI/AnimationPanel.h GUI/OffScreenRenderer.h GUI/widgets/ActionButton.h
	GUI/ProgramState.h
)

SET(kvf2d_HEADERS
	${kvf2d_HEADERS_QT}
	GUI/QVideoEncoder.h
	core/MeshModel.h core/KVFModel.h core/VideoModel.h
	core/BDMORPH.h core/OutlineModel.h
	utils/cholmod_matrix.h utils/cholmod_common.h utils/vector2d.h 
	utils/triangle.h utils/utils.h
)
	
SET(kvf2d_FORMS 
	GUI/MainWindow.ui GUI/SidePanel.ui GUI/AnimationPanel.ui GUI/About.ui
)

SET(kvf2d_RESOURCES  GUI/resources/resources.rc )

###################################################################################################

QT4_WRAP_CPP(kvf2d_HEADERS_MOC ${kvf2d_HEADERS_QT})
QT4_WRAP_UI(kvf2d_FORMS_HEADERS ${kvf2d_FORMS})
QT4_ADD_RESOURCES(kvf2d_RESOURCES_RCC ${kvf2d_RESOURCES})

ADD_EXECUTABLE(kvf2d WIN32 MACOSX_BUNDLE 
	${kvf2d_SOURCES}
	${kvf2d_RESOURCES_RCC} 
	${kvf2d_HEADERS_MOC} 
	${kvf2d_FORMS_HEADERS}
	${kvf2d_HEADERS}
	${ICON_RC_FILE}
)

TARGET_LINK_LIBRARIES(kvf2d 
	${QT_LIBRARIES} 
	${OPENGL_LIBRARIES} 
	${EXTERNAL_LIBRARIES} 
)
###################################################################################################

