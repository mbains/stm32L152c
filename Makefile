
all:
	~/Ac6/SystemWorkbench/eclipse  -nosplash --launcher.suppressErrors -application org.eclipse.cdt.managedbuilder.core.headlessbuild -build 'stm32L152c/Debug'

clean:
	rm -r ./Debug/


