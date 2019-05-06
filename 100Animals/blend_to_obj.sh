#!/bin/sh
#Usage: bash blend_to_obj.sh directory
for file in "$1"/*
do
  extension="${file##*.}"
  filename="${file%.*}"
  newname="${file%_base_mesh_BI.*}"
  if [ $extension == "blend" ] 
  then
    blender --background --python blend_to_obj.py -- "$PWD/$filename.blend" "$PWD/$newname"
  fi
done
