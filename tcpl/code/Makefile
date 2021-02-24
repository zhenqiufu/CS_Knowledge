SUBDIR  =  ./chapter_1
SUBDIR_EXERCISE = ./exercise
MAKE  =  make
MAKECLEAN = make clean
subsystem:
	cd $(SUBDIR)  &&  $(MAKE)
	cd $(SUBDIR)  &&  cd $(SUBDIR_EXERCISE) && $(MAKE)


clean:
	cd $(SUBDIR)  &&  $(MAKECLEAN)
	cd $(SUBDIR)  &&  cd $(SUBDIR_EXERCISE) && $(MAKECLEAN)
