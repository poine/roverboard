#!/usr/bin/env python

import odrive_can_cpp_ext

def main():
    od = odrive_can_cpp_ext.Odrive()
    od.init()

    
if __name__ == '__main__':
    main()
