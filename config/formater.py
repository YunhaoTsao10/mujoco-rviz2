import os
import yaml

def load_config(config_path):
    """加载路径配置文件."""
    if os.path.exists(config_path):
        with open(config_path, 'r') as file:
            return yaml.safe_load(file).get('paths', {})
    else:
        raise FileNotFoundError(f"配置文件 {config_path} 不存在！")

def replace_mesh_paths(urdf_file_path, old_mesh_dir, new_mesh_dir):
    """替换 URDF 文件中 mesh 路径."""
    with open(urdf_file_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()

    # 替换旧路径为新路径
    new_urdf_content = urdf_content.replace(
        f'file://{old_mesh_dir}/',
        f'file://{new_mesh_dir}/'
    )

    with open(urdf_file_path, 'w') as urdf_file:
        urdf_file.write(new_urdf_content)

    print(f"URDF 文件路径已更新: {urdf_file_path}")

if __name__ == "__main__":
    
    # 加载配置文件
    config_file = os.getenv('CONFIG_FILE_PATH')
    print(config_file)
    paths = load_config(config_file)

    # 获取路径
    urdf_file_path = paths.get('urdf_file', '<if urdf_file is not found, type your absolute path here!>')
    old_mesh_dir = paths.get('old_mesh_dir')
    new_mesh_dir = paths.get('new_mesh_dir', '<if new_mesh_dir is not found, type your absolute path here!>')

    if not urdf_file_path or not old_mesh_dir or not new_mesh_dir:
        raise ValueError("配置文件中的路径不完整！请检查 paths.yaml")

    # 调用函数替换路径
    replace_mesh_paths(urdf_file_path, old_mesh_dir, new_mesh_dir)
