pipeline {
    agent {
        docker {
            image 'amiller27/iarc7-base'
            args '-u root'
        }
    }
    stages {
        stage ('Setup-Dependencies') {
            steps {
                sh '''
                    cd $WORKSPACE
                    source /opt/ros/kinetic/setup.bash
                    if [[ ! -f "$WORKSPACE/.rosinstall" ]]; then
                        wstool init
                    fi
                    if [[ -f "$WORKSPACE/dependencies.rosinstall" ]]; then
                        wstool merge "$WORKSPACE/dependencies.rosinstall"
                    fi
                    wstool update
                    apt-get update
                    rosdep install -y --from-paths src --ignore-src --rosdistro kinetic
                    '''
            }
        }
        stage ('Build') {
            steps {
                sh '''
                    cd $WORKSPACE
                    source /opt/ros/kinetic/setup.bash
                    catkin_make clean
                    catkin_make
                    '''
            }
        }
        stage ('Test') {
            steps {
                sh '''
                    cd $WORKSPACE
                    source /opt/ros/kinetic/setup.bash
                    source devel/setup.bash
                    catkin_make test
                    '''
            }
        }
    }
}

