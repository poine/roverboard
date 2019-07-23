
odrive_can_cpp_ext.so: odrive_can_cpp_ext.pyx setup_odrive_can_cpp_ext.py
	python setup_odrive_can_cpp_ext.py build_ext --inplace

odrive_ascii_cpp_ext.so: odrive_ascii_cpp_ext.pyx setup_odrive_ascii_cpp_ext.py
	python setup_odrive_ascii_cpp_ext.py build_ext --inplace

clean:
	rm -rf *~ *.pyc build odrive_can_cpp_ext.cpp odrive_can_cpp_ext.so odrive_ascii_cpp_ext.cpp odrive_ascii_cpp_ext.so
