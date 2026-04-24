from setuptools import find_packages, setup

package_name = 'smart_shop_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ssu',
    maintainer_email='ssu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stock_server = smart_shop_nodes.stock_server:main',
            'payment_server = smart_shop_nodes.payment_server:main',
            'order_manager = smart_shop_nodes.order_manager:main',
            'order_client = smart_shop_nodes.order_client:main',
            'discount_server = smart_shop_nodes.discount_server:main',
            'log_monitor = smart_shop_nodes.log_monitor:main',
        ],
    },
)
