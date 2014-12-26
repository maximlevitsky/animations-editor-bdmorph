cmake_minimum_required(VERSION 2.6)

PROJECT(bdmorph)
FIND_PACKAGE(Qt4 REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
SET(QT_USE_QTOPENGL TRUE)
INCLUDE(${QT_USE_FILE})

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING   
        "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
        FORCE)
endif(NOT CMAKE_BUILD_TYPE)

###################################################################################################

if (WIN32)
	SET(ROOT C:/Users/mlevtsky/Workspace/external)
	SET (EXTERNAL_LIBRARIES
		${ROOT}/lib/libamd.lib
		${ROOT}/lib/libcamd.lib
		${ROOT}/lib/libcolamd.lib
		${ROOT}/lib/libccolamd.lib
		${ROOT}/lib/libcholmod.lib

		${ROOT}/lib/libgoto_CHOLMOD.lib
		${ROOT}/lib/libmetis_CHOLMOD.lib
		
		${ROOT}/lib/avcodec.lib
		${ROOT}/lib/avformat.lib
		${ROOT}/lib/avutil.lib
		${ROOT}/lib/swscale.lib
	)
	
	SET(EXTERNAL_INCLUDES
		${ROOT}/include/
		${ROOT}/include/suitesparse/
	)

	SET(ICON_RC_FILE GUI/icon/winicon.rc)

	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -D_CRT_SECURE_NO_WARNINGS -D_CRT_NONSTDC_NO_DEPRECATE  -DNOMINMAX -D_USE_MATH_DEFINES ")
	set(CMAKE_C_FLAGS   "${CMAKE_CXX_FLAGS}  -D_CRT_SECURE_NO_WARNINGS -D_CRT_NONSTDC_NO_DEPRECATE  -DNOMINMAX -D_USE_MATH_DEFINES ")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /OPT:NOREF" )
	
endif(WIN32)

###################################################################################################

if (APPLE)
	SET (EXTERNAL_LIBRARIES
		cholmod amd camd colamd suitesparseconfig
		blas lapack

		avcodec avformat avutil swscale swresample
		"-framework CoreFoundation"
		"-framework CoreVideo"
		"-framework VideoDecodeAcceleration"
		  x264 z bz2 iconv
	)

	SET (EXTERNAL_INCLUDES
		/usr/local/include/suitesparse
	)

	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-reorder")

	set(MACOSX_BUNDLE_ICON_FILE macicon.icns)

	set(bdmorph_ICON ${CMAKE_CURRENT_SOURCE_DIR}/GUI/icon/macicon.icns)
	set_source_files_properties(${bdmorph_ICON} PROPERTIES
       MACOSX_PACKAGE_LOCATION "Resources")

    SET(ICON_RC_FILE ${bdmorph_ICON})

endif(APPLE)

###################################################################################################

if (NOT APPLE AND UNIX) # <- this is the way it should be
	SET (EXTERNAL_LIBRARIES
		cholmod amd camd colamd
		openblas lapack		
		avcodec avformat avutil swscale
	)

	SET (EXTERNAL_INCLUDES
		/usr/include/suitesparse
	)

	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-reorder -Woverloaded-virtual")
endif(NOT APPLE AND UNIX)

###################################################################################################

INCLUDE_DIRECTORIES(
	${CMAKE_CURRENT_BINARY_DIR} 
	${EXTERNAL_INCLUDES}
	. ./GUI ./core ./utils ./GUI/widgets
)

SET(bdmorph_SOURCES
	GUI/EditorWindow.cpp GUI/Main.cpp  GUI/MainWindow.cpp 
	GUI/SidePanel.cpp GUI/AnimationPanel.cpp GUI/OffScreenRenderer.cpp
	GUI/widgets/ActionButton.cpp GUI/ProgramState.cpp
	GUI/QVideoEncoder.cpp

	core/MeshModel.cpp core/KVFModel.cpp core/VideoModel.cpp
	core/BDMORPH.cpp core/OutlineModel.cpp

	utils/cholmod_matrix.cpp utils/cholmod_common.cpp utils/vector2d.cpp 
	utils/triangle.c utils/utils.cpp
)

SET(bdmorph_HEADERS_QT
	GUI/EditorWindow.h GUI/MainWindow.h GUI/SidePanel.h 
	GUI/AnimationPanel.h GUI/OffScreenRenderer.h GUI/widgets/ActionButton.h
	GUI/ProgramState.h
)

SET(bdmorph_HEADERS
	${bdmorph_HEADERS_QT}
	GUI/QVideoEncoder.h
	core/MeshModel.h core/KVFModel.h core/VideoModel.h
	core/BDMORPH.h core/OutlineModel.h
	utils/cholmod_matrix.h utils/cholmod_common.h utils/vector2d.h 
	utils/triangle.h utils/utils.h
)
	
SET(bdmorph_FORMS 
	GUI/MainWindow.ui GUI/SidePanel.ui GUI/AnimationPanel.ui GUI/About.ui
)

SET(bdmorph_RESOURCES  GUI/resources/resources.rc )

###################################################################################################

QT4_WRAP_CPP(bdmorph_HEADERS_MOC ${bdmorph_HEADERS_QT})
QT4_WRAP_UI(bdmorph_FORMS_HEADERS ${bdmorph_FORMS})
QT4_ADD_RESOURCES(bdmorph_RESOURCES_RCC ${bdmorph_RESOURCES})

ADD_EXECUTABLE(bdmorph WIN32 MACOSX_BUNDLE 
	${bdmorph_SOURCES}
	${bdmorph_RESOURCES_RCC} 
	${bdmorph_HEADERS_MOC} 
	${bdmorph_FORMS_HEADERS}
	${bdmorph_HEADERS}
	${ICON_RC_FILE}
)

TARGET_LINK_LIBRARIES(bdmorph 
	${QT_LIBRARIES}
	${OPENGL_LIBRARIES}
	${EXTERNAL_LIBRARIES}
)


###################################################################################################

