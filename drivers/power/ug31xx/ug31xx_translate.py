#!/usr/bin/env python
#
#  Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
#  Written by wade chen Wade1_Chen@asus.com
# 

"""
 ug31xx_translate.py  [-m [MODE]] <input_file_names> -o <output_file_name>
 when not specify MODE, the default MODE is GGBX2BUF:
   1. <file_name> = <ggbx_file_name>
   2. translate to <output_file_name>, c coding style
 when specify MODE = GGB2GGBX:
   1. <file_name> = <ggb_file>, multi-files support.
   2. translate to .ggbx file
 when specify MODE = GGBX2GGB:
   1. <file_name> = <ggbx_file>, not support multi-files
   2. translate to .ggb file

GGB file header structure:   
typedef struct _GGBX_FILE_HEADER
{
        u32             ggb_tag;        //'_GG_'
        u32             sum16;          //16 bits checksum, but store as 4 bytes
        u64             time_stamp;     //seconds pass since 1970 year, 00 hour, 00 minute, 00 second
        u64             length;         //size that only include ggb content
        u64             num_ggb;        //number of ggb files. 
}  __attribute__((packed)) GGBX_FILE_HEADER;
"""

import getopt
import os, sys
import re
import string
import struct
import shlex
import shutil
import time

class _bits:
  def __init__(self, val, bit_len):
    if bit_len < 8:
      raise RuntimeError, "Bit length too small"
    if type(val) is not int:
      raise RuntimeError, "Value must be integer."

    self._val = val
    self._bits = bit_len
    self._mask = (1<<bit_len) - 1

  def __add__(self, val):
    return _bits((((int)(self._val + val)) & self._mask), self._bits)

  def __radd__(self, val):
    return _bits((((int)(self._val + val)) & self._mask), self._bits)

  def __invert__(self):
    print "invert %X %X" % (~self._val, ~self._val & self._mask)
    return _bits( (int)(~self._val & self._mask), self._bits)


  def __int__(self):
    return self._val

class uint8(_bits):
  def __init__(self, val):
    _bits.__init__(self, val, 8)

class uint16(_bits):
  def __init__(self, val):
    _bits.__init__(self, val, 16)

class uint32(_bits):
  def __init__(self, val):
    _bits.__init__(self, val, 32)



def do_ggbx2buf(in_file, out_file):

  if len(in_file) > 1:
    raise RuntimeError, "Not support multi-files."

  with open(in_file[0], mode="rb") as f:
    try:
      in_data_buf = f.read()
    except IOError:
      raise RuntimeError, "Reading %s error." % in_file[0]
    finally:
      f.close()

  title = """
/* This file is auto-generated. Don't edit this file. */

char FactoryGGBXFile[] = { 
"""

  remainder = len(in_data_buf) % 8
  remainder_start = 0
  content = ""
  for i in range(0, len(in_data_buf), 8):
    if i+8 >= len(in_data_buf):
      break

    line = [ "0x%02x," % ord(in_data_buf[j]) for j in range(i, i+8) ]
    content = content + "    " + ''.join(line) + "\n"
    remainder_start = i+8

  if remainder != 0:
    line = [ "0x%02x," % ord(in_data_buf[j]) for j in range(remainder_start, remainder_start+remainder) ]
    content = content + "    " + ''.join(line) 

  end_title = """
};
  """

  out_buf = title + content + end_title

  try:
    with open(out_file, mode="wb") as f:
      f.write(out_buf)
  except IOError:
    raise RuntimeError, "Writing %s error." % out_file
  finally:
    f.close()


def do_ggbx2ggb(in_file, out_file):

  if len(in_file) > 1:
    raise RuntimeError, "Not support multi-files."

  with open(in_file[0], mode="rb") as f:
    try:
      in_data_buf = f.read()
    except IOError:
      raise RuntimeError, "Reading %s error." % in_file
    finally:
      f.close()

  #fetch header
  header_size = 4 + 4 + 8 + 8 + 8
  (ggb_tag, sum16, time_stamp, length, num_ggb) = struct.unpack('IIQQQ', in_data_buf[:header_size]) 

  #header checking
  if ggb_tag != 0x5F47475F: # _GG_
    raise RuntimeError, "ggb tag not correct"

  if len(in_data_buf) < length:
    raise RuntimeError, "ggbx structure length not correct. %d, %d" % (len_data_buf, length)

  if num_ggb == 0:
    raise RuntimeError, "Number of ggb file not correct. %d" % num_ggb

  if len(in_data_buf[header_size:]) % num_ggb != 0:
    raise RuntimeError, "ggbx file size not correct."

  # check sum16
  checksum = uint16(0)
  for one_byte in in_data_buf[header_size:]:
    checksum += ord(one_byte)
  
  if sum16 != int(checksum):
    raise RuntimeError, "Checksum fail."

  print "ggb_tag: 0x%08X" % ggb_tag
  print "sum16: 0x%04X" % sum16
  print "time_stamp: 0x%08X" % time_stamp
  print "length: 0x%08X" % length
  print "num of ggb: %d" % num_ggb 

  start2fetch = header_size
  ggb_size = len(in_data_buf[start2fetch:]) // num_ggb
  for idx in range(0, num_ggb):
    start2fetch = header_size + idx * ggb_size
    with open(out_file+".%d" % idx, mode="wb") as f:
      try:
        f.write(in_data_buf[start2fetch: start2fetch+ggb_size])
      except IOError:
        raise RuntimeError, "Writing %s error." % out_file
      finally:
        f.close()



def do_ggb2ggbx(in_file, out_file):
  #print in_file
  #print out_file
  in_data_buf = ""
  for input_file in in_file:
    with open(input_file, mode="rb") as f:
      try:
        _in_data_buf = f.read()
      except IOError:
        raise RuntimeError, "Reading %s error." % in_file
      finally:
        f.close()

    in_data_buf += _in_data_buf

  #add header field
  header_size = 4 + 4 + 8 + 8 + 8
  time_stamp = int(time.time())
  ggb_tag = 0x5F47475F# _GG_
  length = len(in_data_buf) 
  num_ggb = len(in_file)
  #calculate checksum
  checksum = uint16(0)
  for one_byte in in_data_buf:
    checksum += ord(one_byte)
  sum16 = int(checksum)
  out_buf = struct.pack('IIQQQ%ds' % len(in_data_buf), 
              ggb_tag, sum16, time_stamp, length, num_ggb, in_data_buf)

  print "sum16 0x%04X, length = %X, time_stamp %8X, num_ggb %d" \
        % (int(checksum), length, time_stamp, num_ggb)

  with open(out_file, mode="wb") as f:
    try:
      f.write(out_buf)
    except IOError:
      raise RuntimeError, "Writing %s error." % out_file
    finally:
      f.close()



def do_ggb_stuff(argv):
  mode = "GGBX2BUF"
  try:
    opts, args = getopt.getopt(argv, "m:o:") 
  except getopt.GetoptError, err:
    raise RuntimeError, "Argument format incorrect."

  output_file = ""
  for o, a in opts:
    if o == "-m":
      mode = a
    elif o == "-o":
      output_file = a
    else:
      raise RuntimeError, "Unknown argument."

  if mode != "GGB2GGBX" and mode != "GGBX2GGB" and mode != "GGBX2BUF":
    raise RuntimeError, "Unknown mode."

  if len(args) < 1:
    raise RuntimeError, "Arguments not enough."

  input_file = args
  if os.path.exists(args[0]) is False:
    raise RuntimeError, "Input file %s not found." % input_file

  if mode == "GGBX2BUF":
    do_ggbx2buf(input_file, output_file)
  elif mode == "GGBX2GGB":
    do_ggbx2ggb(input_file, output_file)
  elif mode == "GGB2GGBX":
    do_ggb2ggbx(input_file, output_file)


if __name__ == '__main__':
  try:
    do_ggb_stuff(sys.argv[1:])
  except RuntimeError, eString:
    print "%s\n" % eString 
    sys.exit(1)




#hexdump ${bin_file} -v -e ' "        " 8/1 "0x%02X," "\n"' >> ${header_file}
#cat >> ${header_file} << EOF
#};
#EOF
