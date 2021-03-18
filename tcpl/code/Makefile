SUBDIR  =  ./chapter_1 
SUBDIR_3  =  ./chapter_3
SUBDIR_7  =  ./chapter_7

SUBDIR_EXERCISE = ./exercise
MAKE  =  make
MAKECLEAN = make clean
subsystem:
	cd $(SUBDIR)  &&  $(MAKE)
	cd $(SUBDIR)  &&  cd $(SUBDIR_EXERCISE) && $(MAKE)
	cd $(SUBDIR_3)  &&  $(MAKE)
	cd $(SUBDIR_3)  &&  cd $(SUBDIR_EXERCISE) && $(MAKE)
	cd $(SUBDIR_7)  &&  $(MAKE)
	cd $(SUBDIR_7)  &&  cd $(SUBDIR_EXERCISE) && $(MAKE)
	@ echo "***************************"
	@ echo "MAKE ALL SUCCESSFULLY"
	@ echo "***************************"

clean:
	cd $(SUBDIR)  &&  $(MAKECLEAN)
	cd $(SUBDIR)  &&  cd $(SUBDIR_EXERCISE) && $(MAKECLEAN)
	cd $(SUBDIR_3)  &&  $(MAKECLEAN)
	cd $(SUBDIR_3)  &&  cd $(SUBDIR_EXERCISE) && $(MAKECLEAN)
	cd $(SUBDIR_7)  &&  $(MAKECLEAN)
	cd $(SUBDIR_7)  &&  cd $(SUBDIR_EXERCISE) && $(MAKECLEAN)
	@ echo "***************************"
	@ echo "MAKE CLEAN ALL SUCCESSFULLY"
	@ echo "***************************"

