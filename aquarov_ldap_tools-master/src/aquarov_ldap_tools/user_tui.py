from aquarov_ldap_tools.ldap_controller import LdapController
from getpass import getpass
from aquarov_ldap_tools.utils import clear_screen

LDAP_SERVER = "ldap://server1.telemcloud.cl:389"


def main():
    clear_screen()

    print("Aquarov development LDAP change user password.\n")

    # Connect to the LDAP server
    ldap_controller = LdapController(LDAP_SERVER)

    with ldap_controller:

        # User information
        username = input("Username: ")
        old_password = getpass("Current password: ")

        # User authentication
        ldap_controller.log_as_user(username, old_password)
        print("\nValid credentials...\n")

        # Enter new password
        new_password = getpass("New password: ")
        new_password2 = getpass("Repeat new password: ")
        assert new_password == new_password2

        ldap_controller.change_user_password(username, new_password)

        print("La contraseña se ha cambiado exitosamente")


if __name__ == '__main__':
    main()
