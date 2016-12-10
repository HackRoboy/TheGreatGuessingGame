require 'rake/testtask'

task default: :build

desc 'Builds the App.'
task build: 'DialogSystem/DialogSystem.jar'

task :format do
  options = []
  options.push '--replace' if ENV['repair']
  sh "gherkin_format #{options.join ' '} features/*.feature"
end

task run: 'DialogSystem/DialogSystem.jar' do
  sh run_command()
end

def run_command()
  "#{ENV['JAVA_HOME']}/bin/java -cp DialogSystem:DialogSystem/resources:DialogSystem/DialogSystem.jar:#{FileList['DialogSystem/lib/*.jar'].join ':'} de.roboy.dialog.DialogSystem"
end

file 'DialogSystem/DialogSystem.jar' => FileList['DialogSystem/**/*.java'] do
  cd('DialogSystem') do 
    mkdir_p 'package'
    sh 'cp -r resources ..'
    sh "#{ENV['JAVA_HOME']}/bin/javac #{FileList['src/**/*.java']} -cp #{FileList['lib/*.jar'].join ':'} -d package"
    sh "#{ENV['JAVA_HOME']}/bin/jar cfm DialogSystem.jar Manifest.txt -C package/ ."
  end
end

desc 'Tests the Application'
task :test => :build do
  ENV['sut'] = run_command
 
  options = []
  options << '--stop' if ENV['stop']
  options << '--tags ~skip'
  sh "PYTHONPATH=src behave #{options * ' '} test"
end
