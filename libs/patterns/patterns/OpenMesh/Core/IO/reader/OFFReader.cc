/*===========================================================================*\
 *                                                                           *
 *                               OpenMesh                                    *
 *      Copyright (C) 2001-2014 by Computer Graphics Group, RWTH Aachen      *
 *                           www.openmesh.org                                *
 *                                                                           *
 *---------------------------------------------------------------------------*
 *  This file is part of OpenMesh.                                           *
 *                                                                           *
 *  OpenMesh is free software: you can redistribute it and/or modify         *
 *  it under the terms of the GNU Lesser General Public License as           *
 *  published by the Free Software Foundation, either version 3 of           *
 *  the License, or (at your option) any later version with the              *
 *  following exceptions:                                                    *
 *                                                                           *
 *  If other files instantiate templates or use macros                       *
 *  or inline functions from this file, or you compile this file and         *
 *  link it with other files to produce an executable, this file does        *
 *  not by itself cause the resulting executable to be covered by the        *
 *  GNU Lesser General Public License. This exception does not however       *
 *  invalidate any other reasons why the executable file might be            *
 *  covered by the GNU Lesser General Public License.                        *
 *                                                                           *
 *  OpenMesh is distributed in the hope that it will be useful,              *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *  GNU Lesser General Public License for more details.                      *
 *                                                                           *
 *  You should have received a copy of the GNU LesserGeneral Public          *
 *  License along with OpenMesh.  If not,                                    *
 *  see <http://www.gnu.org/licenses/>.                                      *
 *                                                                           *
\*===========================================================================*/

/*===========================================================================*\
 *                                                                           *
 *   $Revision: 1053 $                                                         *
 *   $Date: 2014-05-09 14:44:18 +0200 (金, 09 5 2014) $                   *
 *                                                                           *
\*===========================================================================*/


#define LINE_LEN 4096


//== INCLUDES =================================================================

// OpenMesh
#include <OpenMesh/Core/System/config.h>
#include <OpenMesh/Core/System/omstream.hh>
#include <OpenMesh/Core/IO/reader/OFFReader.hh>
#include <OpenMesh/Core/IO/importer/BaseImporter.hh>
#include <OpenMesh/Core/IO/IOManager.hh>
#include <OpenMesh/Core/Utils/color_cast.hh>
// #include <OpenMesh/Core/IO/BinaryHelper.hh>

#include <OpenMesh/Core/IO/SR_store.hh>

//STL
#include <iostream>
#include <ios>
#include <fstream>
#include <memory>

#if defined(OM_CC_MIPS)
#  include <ctype.h>
/// \bug Workaround for STLPORT 4.6: isspace seems not to be in namespace std!
#elif defined(_STLPORT_VERSION) && (_STLPORT_VERSION==0x460)
#  include <cctype>
#else
#  include <cctype>
using std::isspace;
#endif

#ifndef WIN32
#include <string.h>
#endif

//=== NAMESPACES ==============================================================


namespace OpenMesh {
namespace IO {


//=============================================================================

//=== INSTANCIATE =============================================================


_OFFReader_  __OFFReaderInstance;
_OFFReader_&  OFFReader() { return __OFFReaderInstance; }


//=== IMPLEMENTATION ==========================================================



_OFFReader_::_OFFReader_()
{
  IOManager().register_module(this);
}


//-----------------------------------------------------------------------------


bool
_OFFReader_::read(const std::string& _filename, BaseImporter& _bi,
                  Options& _opt)
{
  std::ifstream ifile(_filename.c_str(), (options_.is_binary() ? std::ios::binary | std::ios::in
                                                           : std::ios::in) );

  if (!ifile.is_open() || !ifile.good())
  {
    omerr() << "[OFFReader] : cannot not open file "
	  << _filename
	  << std::endl;

    return false;
  }

  assert(ifile);

  bool result = read(ifile, _bi, _opt);

  ifile.close();
  return result;
}

//-----------------------------------------------------------------------------


bool
_OFFReader_::read(std::istream& _in, BaseImporter& _bi, Options& _opt )
{
   if (!_in.good())
   {
     omerr() << "[OMReader] : cannot not use stream "
       << std::endl;
     return false;
   }

   // filter relevant options for reading
   bool swap = _opt.check( Options::Swap );


   userOptions_ = _opt;

   // build options to be returned
   _opt.clear();

   if (options_.vertex_has_normal() && userOptions_.vertex_has_normal())     _opt += Options::VertexNormal;
   if (options_.vertex_has_texcoord() && userOptions_.vertex_has_texcoord()) _opt += Options::VertexTexCoord;
   if (options_.vertex_has_color() && userOptions_.vertex_has_color())       _opt += Options::VertexColor;
   if (options_.face_has_color() && userOptions_.face_has_color())           _opt += Options::FaceColor;
   if (options_.is_binary())                                                 _opt += Options::Binary;

   //force user-choice for the alpha value when reading binary
   if ( options_.is_binary() && userOptions_.color_has_alpha() )
     options_ += Options::ColorAlpha;

    return (options_.is_binary() ?
 	   read_binary(_in, _bi, _opt, swap) :
	   read_ascii(_in, _bi, _opt));

}



//-----------------------------------------------------------------------------

bool
_OFFReader_::read_ascii(std::istream& _in, BaseImporter& _bi, Options& _opt) const
{


  unsigned int            i, j, k, l, idx;
  unsigned int            nV, nF, dummy;
  OpenMesh::Vec3f         v, n;
  OpenMesh::Vec2f         t;
  OpenMesh::Vec3i         c3;
  OpenMesh::Vec3f         c3f;
  OpenMesh::Vec4i         c4;
  OpenMesh::Vec4f         c4f;
  BaseImporter::VHandles  vhandles;
  VertexHandle            vh;

  // read header line
  std::string header;
  std::getline(_in,header);

  // + #Vertice, #Faces, #Edges
  _in >> nV;
  _in >> nF;
  _in >> dummy;

  _bi.reserve(nV, 3*nV, nF);

  // read vertices: coord [hcoord] [normal] [color] [texcoord]
  for (i=0; i<nV && !_in.eof(); ++i)
  {
    // Always read VERTEX
    _in >> v[0]; _in >> v[1]; _in >> v[2];

    vh = _bi.add_vertex(v);

    //perhaps read NORMAL
    if ( options_.vertex_has_normal() ){

      _in >> n[0]; _in >> n[1]; _in >> n[2];

      if ( userOptions_.vertex_has_normal() )
        _bi.set_normal(vh, n);
    }

    //take the rest of the line and check how colors are defined
    std::string line;
    std::getline(_in,line);

    int colorType = getColorType(line, options_.vertex_has_texcoord() );

    std::stringstream stream( line );

    //perhaps read COLOR
    if ( options_.vertex_has_color() ){

      std::string trash;

      switch (colorType){
        case 0 : break; //no color
        case 1 : stream >> trash; break; //one int (isn't handled atm)
        case 2 : stream >> trash; stream >> trash; break; //corrupt format (ignore)
        // rgb int
        case 3 : stream >> c3[0];  stream >> c3[1];  stream >> c3[2];
            if ( userOptions_.vertex_has_color() )
              _bi.set_color( vh, Vec3uc( c3 ) );
            break;
        // rgba int
        case 4 : stream >> c4[0];  stream >> c4[1];  stream >> c4[2]; stream >> c4[3];
            if ( userOptions_.vertex_has_color() )
              _bi.set_color( vh, Vec4uc( c4 ) );
            break;
        // rgb floats
        case 5 : stream >> c3f[0];  stream >> c3f[1];  stream >> c3f[2];
            if ( userOptions_.vertex_has_color() ) {
              _bi.set_color( vh, c3f );
              _opt += Options::ColorFloat;
            }
            break;
        // rgba floats
        case 6 : stream >> c4f[0];  stream >> c4f[1];  stream >> c4f[2]; stream >> c4f[3];
            if ( userOptions_.vertex_has_color() ) {
              _bi.set_color( vh, c4f );
              _opt += Options::ColorFloat;
            }
            break;

        default:
            std::cerr << "Error in file format (colorType = " << colorType << ")\n";
            break;
      }
    }
    //perhaps read TEXTURE COORDs
    if ( options_.vertex_has_texcoord() ){
      stream >> t[0]; stream >> t[1];
      if ( userOptions_.vertex_has_texcoord() )
        _bi.set_texcoord(vh, t);
    }
  }

  // faces
  // #N <v1> <v2> .. <v(n-1)> [color spec]
  for (i=0; i<nF; ++i)
  {
    // nV = number of Vertices for current face
    _in >> nV;

    if (nV == 3)
    {
      vhandles.resize(3);
      _in >> j;
      _in >> k;
      _in >> l;

      vhandles[0] = VertexHandle(j);
      vhandles[1] = VertexHandle(k);
      vhandles[2] = VertexHandle(l);
    }
    else
    {
      vhandles.clear();
      for (j=0; j<nV; ++j)
      {
         _in >> idx;
         vhandles.push_back(VertexHandle(idx));
      }
    }

    FaceHandle fh = _bi.add_face(vhandles);

    //perhaps read face COLOR
    if ( options_.face_has_color() ){

      //take the rest of the line and check how colors are defined
      std::string line;
      std::getline(_in,line);

      int colorType = getColorType(line, false );

      std::stringstream stream( line );

      std::string trash;

      switch (colorType){
        case 0 : break; //no color
        case 1 : stream >> trash; break; //one int (isn't handled atm)
        case 2 : stream >> trash; stream >> trash; break; //corrupt format (ignore)
        // rgb int
        case 3 : stream >> c3[0];  stream >> c3[1];  stream >> c3[2];
            if ( userOptions_.face_has_color() )
              _bi.set_color( fh, Vec3uc( c3 ) );
            break;
        // rgba int
        case 4 : stream >> c4[0];  stream >> c4[1];  stream >> c4[2]; stream >> c4[3];
            if ( userOptions_.face_has_color() )
              _bi.set_color( fh, Vec4uc( c4 ) );
            break;
        // rgb floats
        case 5 : stream >> c3f[0];  stream >> c3f[1];  stream >> c3f[2];
            if ( userOptions_.face_has_color() ) {
              _bi.set_color( fh, c3f );
              _opt += Options::ColorFloat;
            }
            break;
        // rgba floats
        case 6 : stream >> c4f[0];  stream >> c4f[1];  stream >> c4f[2]; stream >> c4f[3];
            if ( userOptions_.face_has_color() ) {
              _bi.set_color( fh, c4f );
              _opt += Options::ColorFloat;
            }
            break;

        default:
            std::cerr << "Error in file format (colorType = " << colorType << ")\n";
            break;
      }
    }
  }

  // File was successfully parsed.
  return true;
}


//-----------------------------------------------------------------------------

int _OFFReader_::getColorType(std::string& _line, bool _texCoordsAvailable) const
{
/*
  0 : no Color
  1 : one int (e.g colormap index)
  2 : two items (error!)
  3 : 3 ints
  4 : 3 ints
  5 : 3 floats
  6 : 4 floats

*/
	// Check if we have any additional information here
	if ( _line.size() < 1 )
		return 0;

    //first remove spaces at start/end of the line
    while (_line.size() > 0 && std::isspace(_line[0]))
      _line = _line.substr(1);
    while (_line.size() > 0 && std::isspace(_line[ _line.length()-1 ]))
      _line = _line.substr(0, _line.length()-1);

    //count the remaining items in the line
    size_t found;
    int count = 0;

    found=_line.find_first_of(" ");
    while (found!=std::string::npos){
      count++;
      found=_line.find_first_of(" ",found+1);
    }

    if (!_line.empty()) count++;

    if (_texCoordsAvailable) count -= 2;

    if (count == 3 || count == 4){
      //get first item
      found = _line.find(" ");
      std::string c1 = _line.substr (0,found);

      if (c1.find(".") != std::string::npos){
        if (count == 3)
          count = 5;
        else
          count = 6;
      }
    }
  return count;
}

void _OFFReader_::readValue(std::istream& _in, float& _value) const{
  float32_t tmp;

  restore( _in , tmp, false ); //assuming LSB byte order
  _value = tmp;
}

void _OFFReader_::readValue(std::istream& _in, int& _value) const{
  uint32_t tmp;

  restore( _in , tmp, false ); //assuming LSB byte order
  _value = tmp;
}

void _OFFReader_::readValue(std::istream& _in, unsigned int& _value) const{
  uint32_t tmp;

  restore( _in , tmp, false ); //assuming LSB byte order
  _value = tmp;
}

bool
_OFFReader_::read_binary(std::istream& _in, BaseImporter& _bi, Options& _opt, bool /*_swap*/) const
{
  unsigned int            i, j, k, l, idx;
  unsigned int            nV, nF, dummy;
  OpenMesh::Vec3f         v, n;
  OpenMesh::Vec3i         c;
  OpenMesh::Vec4i         cA;
  OpenMesh::Vec3f         cf;
  OpenMesh::Vec4f         cAf;
  OpenMesh::Vec2f         t;
  BaseImporter::VHandles  vhandles;
  VertexHandle            vh;

  // read header line
  std::string header;
  std::getline(_in,header);

  // + #Vertice, #Faces, #Edges
  readValue(_in, nV);
  readValue(_in, nF);
  readValue(_in, dummy);

  _bi.reserve(nV, 3*nV, nF);

  // read vertices: coord [hcoord] [normal] [color] [texcoord]
  for (i=0; i<nV && !_in.eof(); ++i)
  {
    // Always read Vertex
    readValue(_in, v[0]);
    readValue(_in, v[1]);
    readValue(_in, v[2]);

    vh = _bi.add_vertex(v);

    if ( options_.vertex_has_normal() ) {
      readValue(_in, n[0]);
      readValue(_in, n[1]);
      readValue(_in, n[2]);

      if ( userOptions_.vertex_has_normal() )
        _bi.set_normal(vh, n);
    }

    if ( options_.vertex_has_color() ) {
      if ( userOptions_.color_is_float() ) {
        _opt += Options::ColorFloat;
        //with alpha
        if ( options_.color_has_alpha() ){
          readValue(_in, cAf[0]);
          readValue(_in, cAf[1]);
          readValue(_in, cAf[2]);
          readValue(_in, cAf[3]);

          if ( userOptions_.vertex_has_color() )
            _bi.set_color( vh, cAf );
        }else{

          //without alpha
          readValue(_in, cf[0]);
          readValue(_in, cf[1]);
          readValue(_in, cf[2]);

          if ( userOptions_.vertex_has_color() )
            _bi.set_color( vh, cf );
        }
      } else {
        //with alpha
        if ( options_.color_has_alpha() ){
          readValue(_in, cA[0]);
          readValue(_in, cA[1]);
          readValue(_in, cA[2]);
          readValue(_in, cA[3]);

          if ( userOptions_.vertex_has_color() )
            _bi.set_color( vh, Vec4uc( cA ) );
        }else{
          //without alpha
          readValue(_in, c[0]);
          readValue(_in, c[1]);
          readValue(_in, c[2]);

          if ( userOptions_.vertex_has_color() )
            _bi.set_color( vh, Vec3uc( c ) );
        }
      }
    }

    if ( options_.vertex_has_texcoord()) {
      readValue(_in, t[0]);
      readValue(_in, t[1]);

      if ( userOptions_.vertex_has_texcoord() )
        _bi.set_texcoord(vh, t);
    }
  }

  // faces
  // #N <v1> <v2> .. <v(n-1)> [color spec]
  // So far color spec is unsupported!
  for (i=0; i<nF; ++i)
  {
    readValue(_in, nV);

    if (nV == 3)
    {
      vhandles.resize(3);
      readValue(_in, j);
      readValue(_in, k);
      readValue(_in, l);

      vhandles[0] = VertexHandle(j);
      vhandles[1] = VertexHandle(k);
      vhandles[2] = VertexHandle(l);
    } else {
      vhandles.clear();
      for (j=0; j<nV; ++j)
      {
         readValue(_in, idx);
         vhandles.push_back(VertexHandle(idx));
      }
    }

    FaceHandle fh = _bi.add_face(vhandles);

    //face color
    if ( _opt.face_has_color() ) {
      if ( userOptions_.color_is_float() ) {
        _opt += Options::ColorFloat;
        //with alpha
        if ( options_.color_has_alpha() ){
          readValue(_in, cAf[0]);
          readValue(_in, cAf[1]);
          readValue(_in, cAf[2]);
          readValue(_in, cAf[3]);

          if ( userOptions_.face_has_color() )
            _bi.set_color( fh , cAf );
        }else{
          //without alpha
          readValue(_in, cf[0]);
          readValue(_in, cf[1]);
          readValue(_in, cf[2]);

          if ( userOptions_.face_has_color() )
            _bi.set_color( fh, cf );
        }
      } else {
        //with alpha
        if ( options_.color_has_alpha() ){
          readValue(_in, cA[0]);
          readValue(_in, cA[1]);
          readValue(_in, cA[2]);
          readValue(_in, cA[3]);

          if ( userOptions_.face_has_color() )
            _bi.set_color( fh , Vec4uc( cA ) );
        }else{
          //without alpha
          readValue(_in, c[0]);
          readValue(_in, c[1]);
          readValue(_in, c[2]);

          if ( userOptions_.face_has_color() )
            _bi.set_color( fh, Vec3uc( c ) );
        }
      }
    }

  }

  // File was successfully parsed.
  return true;

}


//-----------------------------------------------------------------------------


bool _OFFReader_::can_u_read(const std::string& _filename) const
{
  // !!! Assuming BaseReader::can_u_parse( std::string& )
  // does not call BaseReader::read_magic()!!!
  if (BaseReader::can_u_read(_filename))
  {
    std::ifstream ifs(_filename.c_str());
    if (ifs.is_open() && can_u_read(ifs))
    {
      ifs.close();
      return true;
    }
  }
  return false;
}


//-----------------------------------------------------------------------------


bool _OFFReader_::can_u_read(std::istream& _is) const
{
  options_.cleanup();

  // read 1st line
  char line[LINE_LEN], *p;
  _is.getline(line, LINE_LEN);
  p = line;

  std::streamsize remainingChars = _is.gcount();

  bool vertexDimensionTooHigh = false;

  // check header: [ST][C][N][4][n]OFF BINARY

  if ( ( remainingChars > 1 ) && ( p[0] == 'S' && p[1] == 'T') )
  { options_ += Options::VertexTexCoord; p += 2; remainingChars -= 2; }

  if ( ( remainingChars > 0 ) && ( p[0] == 'C') )
  { options_ += Options::VertexColor;
    options_ += Options::FaceColor; ++p; --remainingChars; }

  if ( ( remainingChars > 0 ) && ( p[0] == 'N') )
  { options_ += Options::VertexNormal; ++p; --remainingChars; }

  if ( ( remainingChars > 0 ) && (p[0] == '4' ) )
  { vertexDimensionTooHigh = true; ++p; --remainingChars; }

  if ( ( remainingChars > 0 ) && ( p[0] == 'n') )
  { vertexDimensionTooHigh = true; ++p; --remainingChars; }

  if ( ( remainingChars < 3 ) || (!(p[0] == 'O' && p[1] == 'F' && p[2] == 'F')  ) )
    return false;

  p += 4;

  // Detect possible garbage and make sure, we don't have an underflow
  if ( remainingChars >= 4 )
    remainingChars -= 4;
  else
    remainingChars = 0;

  if ( ( remainingChars >= 6 ) && ( strncmp(p, "BINARY", 6) == 0 ) )
    options_+= Options::Binary;

  // vertex Dimensions != 3 are currently not supported
  if (vertexDimensionTooHigh)
    return false;

  return true;
}


//=============================================================================
} // namespace IO
} // namespace OpenMesh
//=============================================================================
