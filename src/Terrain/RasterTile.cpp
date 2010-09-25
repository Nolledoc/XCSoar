/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000, 2001, 2002, 2003, 2004, 2005, 2006, 2007, 2008, 2009

	M Roberts (original release)
	Robin Birch <robinb@ruffnready.co.uk>
	Samuel Gisiger <samuel.gisiger@triadis.ch>
	Jeff Goodenough <jeff@enborne.f2s.com>
	Alastair Harrison <aharrison@magic.force9.co.uk>
	Scott Penrose <scottp@dd.com.au>
	John Wharington <jwharington@gmail.com>
	Lars H <lars_hn@hotmail.com>
	Rob Dunning <rob@raspberryridgesheepfarm.com>
	Russell King <rmk@arm.linux.org.uk>
	Paolo Ventafridda <coolwind@email.it>
	Tobias Lohner <tobias@lohner-net.de>
	Mirek Jezek <mjezek@ipplc.cz>
	Max Kellermann <max@duempel.org>
	Tobias Bieniek <tobias.bieniek@gmx.de>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "Terrain/RasterTile.hpp"
#include "jasper/jas_image.h"
#include "Math/Angle.hpp"

#include <algorithm>

using std::min;
using std::max;

void
RasterTile::Enable()
{
  if (!width || !height) {
    Disable();
  } else {
    buffer.resize(width, height);
  }
}

short
RasterTile::GetField(unsigned lx, unsigned ly, unsigned ix, unsigned iy) const
{
  // we want to exit out of this function as soon as possible
  // if we have the wrong tile

  if (IsDisabled())
    return RasterBuffer::TERRAIN_INVALID;

  // check x in range
  if ((lx -= xstart) >= width)
    return RasterBuffer::TERRAIN_INVALID;

  // check y in range
  if ((ly -= ystart) >= height)
    return RasterBuffer::TERRAIN_INVALID;

  return buffer.get_interpolated(lx, ly, ix, iy);
}

bool
RasterTile::CheckTileVisibility(const int view_x, const int view_y)
{
  if (!width || !height) {
    Disable();
    return false;
  }

  const unsigned int dx1 = abs(view_x - xstart);
  const unsigned int dx2 = abs(xend - view_x);
  const unsigned int dy1 = abs(view_y - ystart);
  const unsigned int dy2 = abs(yend - view_y);

  if (min(dx1, dx2) * 2 < width * 3) {
    if (min(dy1, dy2) < height)
      return true;
  }
  if (min(dy1, dy2) * 2 < height * 3) {
    if (min(dx1, dx2) < width)
      return true;
  }
  if (IsEnabled()) {
    if ((max(dx1, dx2) > width * 2) || (max(dy1, dy2) > height * 2))
      Disable();
  }
  return false;
}

bool
RasterTile::VisibilityChanged(int view_x, int view_y)
{
  request = CheckTileVisibility(view_x, view_y) && IsDisabled();
  // JMW note: order of these is important!
  return request;
}

short*
RasterTileCache::GetImageBuffer(unsigned index)
{
  if (TileRequest(index))
    return tiles[index].GetImageBuffer();

  return NULL;
}

void
RasterTileCache::SetTile(unsigned index,
                         int xstart, int ystart, int xend, int yend)
{
  if (index >= MAX_RTC_TILES)
    return;

  if (!segments.empty() && segments.last().tile < 0)
    /* link current marker segment with this tile */
    segments.last().tile = index;

  tiles[index].set(xstart, ystart, xend, yend);
}

bool
RasterTileCache::PollTiles(int x, int y)
{
  bool retval = false;
  int i;

  if (scan_overview)
    return false;

  ActiveTiles.clear();

  for (i = MAX_RTC_TILES - 1; --i >= 0;) {
    if (tiles[i].VisibilityChanged(x, y))
      retval = true;

    if (tiles[i].IsEnabled() && !ActiveTiles.full())
      ActiveTiles.append(tiles[i]);
  }

  return retval;
}

bool
RasterTileCache::TileRequest(unsigned index)
{
  unsigned num_used = 0;

  if (index >= MAX_RTC_TILES) {
    // tile index too big!
    return false;
  }

  if (!tiles[index].is_requested())
    return false;

  for (unsigned i = 0; i < MAX_RTC_TILES; ++i)
    if (tiles[i].IsEnabled())
      num_used++;

  if (num_used < MAX_ACTIVE_TILES) {
    tiles[index].Enable();
    return true; // want to load this one!
  }

  return false; // not enough memory for it or not visible anyway
}

short
RasterTileCache::GetField(unsigned int lx, unsigned int ly) const
{
  if ((lx >= overview_width_fine) || (ly >= overview_height_fine))
    // outside overall bounds
    return RasterBuffer::TERRAIN_INVALID;

  unsigned px = lx, py = ly;
  const unsigned int ix = CombinedDivAndMod(px);
  const unsigned int iy = CombinedDivAndMod(py);

  for (unsigned i = 0; i < ActiveTiles.length(); ++i) {
    short h = ActiveTiles[i].GetField(px, py, ix, iy);
    if (h != RasterBuffer::TERRAIN_INVALID) {
      ActiveTiles.move_to_front(i);
      return h;
    }
  }
  // still not found, so go to overview
  return Overview.get_interpolated(lx / RTC_SUBSAMPLING, ly / RTC_SUBSAMPLING);
}

void
RasterTileCache::SetSize(unsigned _width, unsigned _height)
{
  width = _width;
  height = _height;

  Overview.resize(width / RTC_SUBSAMPLING, height / RTC_SUBSAMPLING);
  overview_width_fine = width * 256;
  overview_height_fine = height * 256;
}

void
RasterTileCache::SetLatLonBounds(double _lon_min, double _lon_max,
                                 double _lat_min, double _lat_max)
{

  bounds.west = Angle::degrees(fixed(min(_lon_min, _lon_max)));
  bounds.east = Angle::degrees(fixed(max(_lon_min, _lon_max)));
  bounds.north = Angle::degrees(fixed(max(_lat_min, _lat_max)));
  bounds.south = Angle::degrees(fixed(min(_lat_min, _lat_max)));
}

void
RasterTileCache::Reset()
{
  width = 0;
  height = 0;
  initialised = false;
  segments.clear();
  scan_overview = true;

  Overview.reset();

  for (unsigned i = 0; i < MAX_RTC_TILES; i++)
    tiles[i].Disable();

  ActiveTiles.clear();
}

void
RasterTileCache::SetInitialised(bool val)
{
  if (!initialised && val) {
    if (bounds.empty())
      return;

    initialised = true;
    scan_overview = false;

    return;
  }
  initialised = val;
}

gcc_pure
const RasterTileCache::MarkerSegmentInfo *
RasterTileCache::FindMarkerSegment(long file_offset) const
{
  for (const MarkerSegmentInfo *p = segments.begin(); p < segments.end(); ++p)
    if (p->file_offset >= file_offset)
      return p;

  return NULL;
}

long
RasterTileCache::SkipMarkerSegment(long file_offset) const
{
  if (scan_overview)
    /* use all segments when loading the overview */
    return 0;

  const MarkerSegmentInfo *segment = FindMarkerSegment(file_offset);
  if (segment == NULL)
  if (segment == NULL)
    /* past the end of the recorded segment list; shouldn't happen */
    return 0;

  long skip_to = segment->file_offset;
  while (segment->tile >= 0 && !tiles[segment->tile].is_requested()) {
    ++segment;
    if (segment >= segments.end())
      /* last segment is hidden; shouldn't happen either, because we
         expect EOC there */
      break;

    skip_to = segment->file_offset;
  }

  return skip_to - file_offset;
}

void
RasterTileCache::MarkerSegment(long file_offset, unsigned id)
{
  if (!scan_overview || segments.full())
    return;

  int tile = -1;
  if (id == 0xff93 && !segments.empty())
    /* this SOD segment belongs to the same tile as the preceding SOT
       segment */
    tile = segments.last().tile;

  segments.append(MarkerSegmentInfo(file_offset, tile));
}

extern RasterTileCache *raster_tile_current;

void
RasterTileCache::LoadJPG2000(const char *jp2_filename)
{
  jas_stream_t *in;

  raster_tile_current = this;

  in = jas_stream_fopen(jp2_filename, "rb");
  if (!in) {
    SetInitialised(false);
  } else {
    jp2_decode(in, scan_overview ? "xcsoar=2" : "xcsoar=1");
    jas_stream_close(in);
  }
}

bool
RasterTileCache::LoadOverview(const char *path)
{
  Reset();

  LoadJPG2000(path);
  if (!initialised)
    Reset();

  return initialised;
}

void
RasterTileCache::UpdateTiles(const char *path, int x, int y)
{
  if (PollTiles(x, y)) {
    LoadJPG2000(path);
    PollTiles(x, y);
  }
}
