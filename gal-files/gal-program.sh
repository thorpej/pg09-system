#!/bin/sh -
#
# Copyright (c) 2023 Jason R. Thorpe.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#

#
# gal-program --
#
# A wrapper around minipro for programming, verifying, and dumping
# GAL devices.
#

minipro="minipro"
progname=`basename $0`
action="program"
galtypes="16V8 20V8 22V10"

help()
{
	cat >&2 <<USAGEMESSAGE
usage: $progname [-v] type fusemap.jed # program (or verify) device
       $progname -d type output.jed    # dump device
       $progname -l [filter]           # list GAL device types
       $progname -h                    # display this message (help)
USAGEMESSAGE
}

usage()
{
	help
	exit 1
}

setaction()
{
	if test x"$action" != x"program" -a x"$action" != x"$1"; then
		usage
	fi
	action="$1"
}

action_dump()
{
	if test -f "$jedfile"; then
		echo "ERROR: '$jedfile' exists; move it out of the way!"
		exit 1
	fi
	$minipro -p "$galtype" -r $"jedfile"
}

action_help()
{
	help
}

action_list()
{
	listing=`$minipro -l`
	filter="$galtype"

	for galtype in $galtypes; do
		if test x"filter" != x; then
			echo "$listing" | grep "$galtype" | grep -i "$filter"
		else
			echo "$listing" | grep "$galtype"
		fi
	done
}

action_program()
{
	# Explicitly erase the device first.
	# (XXX Don't actually need to do this ... -w does it, too.)
	# $minipro -p "$galtype" -E

	#
	# At least on my GAL22V10D parts, I need to specify
	# "Do NOT enable write-protect", otherwise writing
	# do the device is unreliable!
	#
	$minipro -p "$galtype" -P -w "$jedfile"

	# Now explicitly verify the device programmed correctly.
	$minipro -p "$galtype" -m "$jedfile"
}

action_verify()
{
	$minipro -p "$galtype" -m "$jedfile"
}

while getopts dhlv opt; do
	case $opt in
	d)	setaction dump ;;
	h)	setaction help ;;
	l)	setaction list ;;
	v)	setaction verify ;;
	\?)	usage ;;
	esac
done
shift $((OPTIND - 1))

case $action in
dump | program | verify)
	galtype="$1"
	jedfile="$2"
	shift 2

	if test x"$galtype" = x -o x"$jedfile" = x; then
		usage
	fi
	;;

list)
	galtype="$1"
	shift 1
	;;
esac

if test x"$1" != x; then
	usage
fi

action_$action

exit 0
