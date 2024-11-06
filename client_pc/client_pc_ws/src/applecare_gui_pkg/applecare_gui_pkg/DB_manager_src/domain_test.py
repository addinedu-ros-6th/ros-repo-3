import subprocess
import os

def run_domain_bridge(yaml_files):
    processes = []
    for yaml_file in yaml_files:
        # 경로 확장: '~'를 사용하여 홈 디렉토리 경로로 변환합니다.
        expanded_yaml_file = os.path.expanduser(yaml_file)
        # subprocess를 사용하여 노드를 실행합니다.
        process = subprocess.Popen(['ros2', 'run', 'domain_bridge', 'domain_bridge', expanded_yaml_file])
        processes.append(process)
    
    return processes
def main():
    # YAML 파일 경로를 리스트로 정의합니다.
    yaml_files = [
        '~/my_test_bridge.yaml',    # 도메인 1에서 2로 전송
        '~/my_test_bridge_2.yaml',   # 도메인 1에서 3으로 전송
        '~/my_test_bridge_service.yaml',   # 도메인 1에서 3으로 전송
        '~/my_test_bridge_service2.yaml'   # 도메인 1에서 3으로 전송
    ]

    # 브릿지를 실행합니다.
    processes = run_domain_bridge(yaml_files)

    try:
        # 모든 프로세스가 종료될 때까지 대기합니다.
        for process in processes:
            process.wait()
    except KeyboardInterrupt:
        # 사용자 입력으로 종료할 경우 모든 프로세스를 종료합니다.
        for process in processes:
            process.terminate()

if __name__ == "__main__":
    main()
