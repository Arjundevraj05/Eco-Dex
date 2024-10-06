'use client';
import React from 'react';
import { Button } from '@nextui-org/react'; // Import Button from Next UI
import { useRouter } from 'next/navigation';
import { LogoutIcon } from '@heroicons/react/outline'; // Import an icon from Heroicons

const LogoutButton = () => {
  const router = useRouter();

  const handleLogout = () => {
    // Clear user authentication tokens
    localStorage.removeItem('token'); // Change 'token' to your actual token name
    sessionStorage.removeItem('token'); // If you're using session storage

    // Redirect to login page or home page after logout
    router.push('/signin'); // Change this to your desired route
  };

  return (
    <Button
      onClick={handleLogout}
      className="w-full mb-4 flex items-center justify-center transition-transform duration-200 transform hover:scale-105 bg-gradient-to-r from-[#388e3c] to-[#47b44c] text-white font-medium rounded-lg py-3 hover:opacity-80"
    >
      <LogoutIcon className="h-5 w-5 mr-2" /> {/* Add icon here */}
      Logout
    </Button>
  );
};

export default LogoutButton;
