# 用于拷贝指定的依赖库


# 定义拷贝函数，参数为原始的库目录，需要替换的库目录，需要拷贝的文件列表
function(copy_libs ORIGIN_LIB_DIR DEST_SUB_DIR DEBUG_DLL_SUFFIX)
    # 设置拷贝的目标名称
    set(LIBS ${ARGN})

    # 正则替换路路径，替换为对应的子目录
    string(REGEX REPLACE "/[^/]+$" "/${DEST_SUB_DIR}" DEST_BIN_LIB_DIR "${ORIGIN_LIB_DIR}")

    # 如果目标名称为空则直接拷贝目录下DEST_BIN_LIB_DIR指定后缀的文件
    if (NOT LIBS)
        if (CMAKE_BUILD_TYPE MATCHES "Debug")
            file(GLOB LIBS "${DEST_BIN_LIB_DIR}/*${DEBUG_DLL_SUFFIX}")
        else ()
            file(GLOB LIBS "${DEST_BIN_LIB_DIR}/*.dll")
        endif ()

        foreach (LIB ${LIBS})
            message(STATUS "Copy ${DEST_BIN_LIB_DIR}/${LIB} to $<TARGET_FILE_DIR:${PROJECT_NAME}>")
            add_custom_command(TARGET ${COPY_TARGET_NAME} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${LIB}"
                    $<TARGET_FILE_DIR:${COPY_TARGET_NAME}>
            )
        endforeach (LIB)
    else ()
        foreach (LIB ${LIBS})
            # 判断是否是debug模式，如果是debug模式则拷贝带d后缀的库
            if (CMAKE_BUILD_TYPE MATCHES "Debug")
                string(REGEX REPLACE ".dll" "${DEBUG_DLL_SUFFIX}" LIB "${LIB}")
            endif (CMAKE_BUILD_TYPE MATCHES "Debug")

            message(STATUS "Copy ${DEST_BIN_LIB_DIR}/${LIB} to $<TARGET_FILE_DIR:${PROJECT_NAME}>")
            add_custom_command(TARGET ${COPY_TARGET_NAME} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${DEST_BIN_LIB_DIR}/${LIB}"
                    $<TARGET_FILE_DIR:${COPY_TARGET_NAME}>
            )

        endforeach (LIB)
    endif ()
endfunction(copy_libs)

# 拷贝PCL库
copy_libs(${PCL_LIBRARY_DIRS} "bin" "d.dll"
        pcl_common.dll
        pcl_search.dll
        pcl_segmentation.dll
        pcl_features.dll
        pcl_filters.dll
        pcl_io.dll
        pcl_kdtree.dll
        pcl_sample_consensus.dll
        pcl_ml.dll
        pcl_octree.dll
        pcl_io_ply.dll
        pcl_outofcore.dll
)


## 拷贝VTK库: 由于基本所有的vtk的都需要，所有直接拷贝所有的
copy_libs(${VTK_RUNTIME_LIBRARY_DIRS} "bin" "-gd.dll")

#copy_libs(${VTK_RUNTIME_LIBRARY_DIRS} "bin" "-gd.dll"
#        vtkIOGeometry-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkIOLegacy-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkIOPLY-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkIOImage-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkIOCore-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkImagingCore-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkCommonExecutionModel-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkCommonDataModel-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkCommonCore-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkCommonColor-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkCommonSystem-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkCommonTransforms-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkCommonComputationalGeometry-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtksys-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkzlib-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkCommonMisc-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtklzma-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtklz4-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkDICOMParser-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkmetaio-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtktiff-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkpng-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkCommonMath-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkjpeg-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkdoubleconversion-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkInteractionStyle-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkRenderingFreeType-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkRenderingOpenGL2-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkRenderingCore-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkglew-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkfreetype-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkFiltersExtraction-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkFiltersSources-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkFiltersCore-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkFiltersGeneral-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkFiltersGeometry-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#        vtkFiltersGeneral-${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.dll
#)

# 拷贝OpenNI2库
copy_libs(${OPENNI2_REDIST_DIR} "Redist" ".dll"
        OpenNI2.dll
)

# 拷贝自定义库
get_filename_component(PARENT_DIR ${CMAKE_CURRENT_LIST_DIR} DIRECTORY)
copy_libs(${PARENT_DIR}/3rd/scan_recognition/lib "lib" ".dll"
#        scan_recognition.dll
)

# 拷贝boost库: boost是静态库不需要拷贝
#copy_libs(${Boost_LIBRARY_DIRS} "bin"
#)



