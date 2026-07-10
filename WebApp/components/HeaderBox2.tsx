// components/HeaderBox.tsx
'use client';

import React from 'react';

import { useEffect, useState } from 'react';





const HeaderBox: React.FC  = () => {
    const [username, setUsername] = useState<string | null>(null);
    useEffect(() => {
        const fetchUsername = async () => {
            try {
                const response = await fetch('/api/user');
                if (response.ok) {
                    const data = await response.json();
                    setUsername(data.username);
                } else {
                    console.error('Failed to fetch username');
                }
            } catch (error) {
                console.error('Error fetching username:', error);
            }
        };

        fetchUsername();
    }, []);
  return (
    <div className="w-full border-b px-4 py-4 sm:px-6 sm:py-6">
      <div>
        <h1 className="text-xl font-poppins font-semibold text-gray-800 sm:text-2xl lg:text-3xl">
          Welcome Back,  
          <span className="bg-gradient-to-r from-green-400 to-green-600 bg-clip-text text-transparent font-poppins">
            {username ? ` ${username}` : ' Guest'}
          </span>!
        </h1>
        <p className="mt-2 text-sm font-normal text-gray-600 sm:text-base">
          Ready to make a difference? Let’s clean up the world together.
        </p>
      </div>
    </div>
  );
};

export default HeaderBox;
