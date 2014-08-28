#!/usr/bin/env python

from optparse import OptionParser


parser = OptionParser()
parser.add_option("-e", "--exec", dest="filename",
                  help="execute plan", metavar="FILE")
parser.add_option("-q", "--quiet",
                  action="store_false", dest="verbose", default=True,
                  help="don't print status messages to stdout")


if __name__ == "__main__":
    (options, args) = parser.parse_args()


