SUBDIR  =  ./chapter_1 
SUBDIR_7  =  ./chapter_7 
SUBDIR_EXERCISE = ./exercise
MAKE  =  make
MAKECLEAN = make clean
subsystem:
	cd $(SUBDIR)  &&  $(MAKE)
	cd $(SUBDIR)  &&  cd $(SUBDIR_EXERCISE) && $(MAKE)
	cd $(SUBDIR_7)  &&  $(MAKE)
	cd $(SUBDIR)  &&  cd $(SUBDIR_EXERCISE) && $(MAKE)

clean:
	cd $(SUBDIR)  &&  $(MAKECLEAN)
	cd $(SUBDIR)  &&  cd $(SUBDIR_EXERCISE) && $(MAKECLEAN)
	cd $(SUBDIR_7)  &&  $(MAKECLEAN)
	cd $(SUBDIR)  &&  cd $(SUBDIR_EXERCISE) && $(MAKECLEAN)